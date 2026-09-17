// SPDX-License-Identifier: MIT
#include "internal.hpp"

namespace irisva {
namespace {
struct Obu {
    unsigned type;
    size_t begin, payload, end;
};

std::vector<Obu> parse_obus(const std::vector<uint8_t> &bytes) {
    std::vector<Obu> result;
    size_t pos = 0;
    while (pos < bytes.size()) {
        size_t begin = pos;
        uint8_t header = bytes[pos++];
        check(!(header & 0x81) && (header & 0x02), "AV1 low-overhead OBU header required",
              VA_STATUS_ERROR_INVALID_BUFFER);
        unsigned type = (header >> 3) & 15;
        if (header & 0x04) {
            check(pos < bytes.size(), "truncated AV1 OBU extension",
                  VA_STATUS_ERROR_INVALID_BUFFER);
            ++pos;
        }
        uint64_t size = 0;
        unsigned shift = 0;
        for (;;) {
            check(pos < bytes.size() && shift < 56, "invalid AV1 OBU size",
                  VA_STATUS_ERROR_INVALID_BUFFER);
            uint8_t byte = bytes[pos++];
            size |= uint64_t(byte & 0x7f) << shift;
            if (!(byte & 0x80))
                break;
            shift += 7;
        }
        check(size <= bytes.size() - pos, "truncated AV1 OBU", VA_STATUS_ERROR_INVALID_BUFFER);
        result.push_back({type, begin, pos, pos + size_t(size)});
        pos += size_t(size);
    }
    return result;
}

} // namespace

std::vector<uint8_t> av1_show_existing(unsigned slot) {
    check(slot < 8, "invalid AV1 reference slot", VA_STATUS_ERROR_INVALID_PARAMETER);
    return {0x12, 0x00, 0x1a, 0x01, uint8_t(0x80 | (slot << 4))};
}

std::vector<uint8_t> av1_bitstream(VAProfile profile, const Av1Picture &picture,
                                   unsigned fourcc, Av1State &state) {
    check(profile == VAProfileAV1Profile0 && picture.has_params &&
              picture.data.size() == 1 && !picture.slices.empty(),
          "AV1 requires one complete coded frame", VA_STATUS_ERROR_INVALID_BUFFER);
    const auto &p = picture.params;
    check(p.profile == 0 && p.bit_depth_idx <= 1 && p.seq_info_fields.fields.subsampling_x &&
              p.seq_info_fields.fields.subsampling_y && !p.pic_info_fields.bits.large_scale_tile,
          "AV1 supports Profile 0 8/10-bit 4:2:0 without large-scale tiles",
          VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
    check((p.bit_depth_idx == 1) == (fourcc == VA_FOURCC_P010),
          "AV1 bit depth does not match output surface", VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT);
    check(!p.film_grain_info.film_grain_info_fields.bits.apply_grain,
          "AV1 film-grain display surfaces are not implemented", VA_STATUS_ERROR_UNIMPLEMENTED);
    check(p.tile_cols && p.tile_rows &&
              picture.slices.size() == size_t(p.tile_cols) * p.tile_rows,
          "AV1 tile parameters do not cover the frame", VA_STATUS_ERROR_INVALID_BUFFER);

    const auto &bytes = picture.data.front();
    check(!bytes.empty(), "empty AV1 coded frame", VA_STATUS_ERROR_INVALID_BUFFER);
    for (const auto &tile : picture.slices) {
        check(tile.slice_data_flag == VA_SLICE_DATA_FLAG_ALL && tile.tile_row < p.tile_rows &&
                  tile.tile_column < p.tile_cols && tile.slice_data_offset <= bytes.size() &&
                  tile.slice_data_size <= bytes.size() - tile.slice_data_offset,
              "invalid AV1 tile data", VA_STATUS_ERROR_INVALID_BUFFER);
    }
    auto obus = parse_obus(bytes);
    std::vector<size_t> selected;
    size_t frame_header = obus.size();
    size_t selected_frame_header = obus.size();
    bool sequence_changed = false;
    for (size_t i = 0; i < obus.size(); ++i) {
        const auto &obu = obus[i];
        if (obu.type == 1) {
            std::vector<uint8_t> sequence(bytes.begin() + obu.begin, bytes.begin() + obu.end);
            if (sequence != state.sequence) {
                state.sequence = std::move(sequence);
                sequence_changed = true;
            }
        } else if (obu.type == 3) {
            frame_header = i;
        }
        for (const auto &tile : picture.slices) {
            size_t begin = tile.slice_data_offset;
            size_t end = begin + tile.slice_data_size;
            if (begin >= obu.payload && end <= obu.end && (obu.type == 4 || obu.type == 6)) {
                if (selected.empty() && obu.type == 4)
                    selected_frame_header = frame_header;
                if (selected.empty() || selected.back() != i)
                    selected.push_back(i);
                break;
            }
        }
    }
    check(!selected.empty(), "AV1 tile data is not contained in a frame/tile-group OBU",
          VA_STATUS_ERROR_INVALID_BUFFER);
    bool frame_obu = obus[selected.front()].type == 6;
    check(frame_obu || selected_frame_header < selected.front(), "AV1 frame header OBU missing",
          VA_STATUS_ERROR_INVALID_BUFFER);

    // Each VA picture becomes an independent stateful-decoder temporal unit.
    // Preserve that boundary even when Chromium's input span contains several
    // frame OBUs from one container block.
    std::vector<uint8_t> out{0x12, 0x00}; // OBU_TEMPORAL_DELIMITER, empty payload
    if (!state.started || sequence_changed) {
        check(!state.sequence.empty(), "AV1 sequence header OBU missing",
              VA_STATUS_ERROR_INVALID_BUFFER);
        out.insert(out.end(), state.sequence.begin(), state.sequence.end());
    }
    if (!frame_obu)
        out.insert(out.end(), bytes.begin() + obus[selected_frame_header].begin,
                   bytes.begin() + obus[selected_frame_header].end);
    for (size_t i : selected) {
        const auto &selected_obu = obus[i];
        out.insert(out.end(), bytes.begin() + selected_obu.begin, bytes.begin() + selected_obu.end);
    }
    state.started = true;
    trace("AV1 OBU input=%zu count=%zu selected=%zu type=%u range=%zu..%zu sequence=%zu",
          bytes.size(), obus.size(), selected.size(), obus[selected.front()].type,
          obus[selected.front()].begin, obus[selected.back()].end, state.sequence.size());
    trace("AV1 frame profile=0 depth=%u tiles=%ux%u type=%u show=%u showable=%u bytes=%zu",
          p.bit_depth_idx ? 10 : 8, p.tile_cols, p.tile_rows,
          p.pic_info_fields.bits.frame_type, p.pic_info_fields.bits.show_frame,
          p.pic_info_fields.bits.showable_frame, out.size());
    return out;
}
} // namespace irisva
