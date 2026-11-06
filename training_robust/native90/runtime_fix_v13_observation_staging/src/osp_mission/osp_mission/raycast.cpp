// Sample the exact existing 2D ray model; no fast-math or fused arithmetic.
#include <cmath>
#include <cstdint>

extern "C" void osp_raycast(
    const std::uint8_t* truth, std::int8_t* observed, int height, int width,
    double x, double y, double resolution,
    const double* cosine, const double* sine, int beams,
    const double* ranges, int samples,
    std::int32_t* end_y, std::int32_t* end_x, std::int32_t* first,
    std::uint8_t* end_inside, std::uint8_t* end_hits) {
    for (int beam = 0; beam < beams; ++beam) {
        for (int sample = 0; sample < samples; ++sample) {
            int col = static_cast<int>(std::floor((x + cosine[beam] * ranges[sample]) / resolution));
            int row = static_cast<int>(std::floor((y + sine[beam] * ranges[sample]) / resolution));
            const bool inside = row >= 0 && row < height && col >= 0 && col < width;
            const int cy = row < 0 ? 0 : (row >= height ? height - 1 : row);
            const int cx = col < 0 ? 0 : (col >= width ? width - 1 : col);
            const bool occupied = truth[cy * width + cx] != 0;
            if (inside) observed[cy * width + cx] = occupied ? 100 : 0;
            if (occupied || !inside || sample == samples - 1) {
                end_y[beam] = cy; end_x[beam] = cx; first[beam] = sample;
                end_inside[beam] = inside; end_hits[beam] = occupied || !inside;
                break;
            }
        }
    }
}
