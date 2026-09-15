import unittest

from task3_six_fast.color_detection import (
    build_sort_jobs,
    classify_roi,
    project_grids,
    stable_colours,
)


class ColourDetectionTests(unittest.TestCase):
    def test_camera_projection_matches_six_grid_arc(self):
        pixels = project_grids(
            {"P1": [0.136387, 0.092863], "P6": [-0.135160, 0.094640]},
            [640, 480], 1.05, 1.35, 0.45,
        )
        self.assertAlmostEqual(pixels["P1"][0], 263.1, delta=1.0)
        self.assertAlmostEqual(pixels["P1"][1], 156.5, delta=1.0)
        self.assertAlmostEqual(pixels["P6"][1], 322.7, delta=1.0)

    def test_rgb_and_bgr_rois_are_classified(self):
        width, height = 20, 20
        yellow_rgb = bytes([245, 220, 20] * width * height)
        green_bgr = bytes([30, 220, 20] * width * height)
        yellow = classify_roi(yellow_rgb, width, height, width * 3, "rgb8", (10, 10), 5)
        green = classify_roi(green_bgr, width, height, width * 3, "bgr8", (10, 10), 5)
        self.assertEqual(yellow.colour, "YELLOW")
        self.assertEqual(green.colour, "GREEN")

    def test_low_colour_count_is_unknown(self):
        grey = bytes([100, 100, 100] * 10 * 10)
        result = classify_roi(grey, 10, 10, 30, "rgb8", (5, 5), 4)
        self.assertIsNone(result.colour)

    def test_stability_requires_consecutive_equal_votes(self):
        self.assertIsNone(stable_colours({"P1": ["YELLOW", "GREEN"]}, 2))
        self.assertEqual(
            stable_colours({"P1": [None, "YELLOW", "YELLOW"]}, 2),
            {"P1": "YELLOW"},
        )

    def test_detected_colours_choose_slots_instead_of_fixed_pairing(self):
        colours = {
            "P1": "GREEN", "P2": "YELLOW", "P3": "GREEN",
            "P4": "YELLOW", "P5": "GREEN", "P6": "YELLOW",
        }
        self.assertEqual(
            build_sort_jobs(colours),
            (
                ("P1", "GREEN_1", "GREEN"),
                ("P2", "YELLOW_1", "YELLOW"),
                ("P3", "GREEN_2", "GREEN"),
                ("P4", "YELLOW_2", "YELLOW"),
                ("P5", "GREEN_3", "GREEN"),
                ("P6", "YELLOW_3", "YELLOW"),
            ),
        )

    def test_full_synthetic_camera_frame_reads_a_shuffled_layout(self):
        grids = {
            "P1": [0.136387, 0.092863], "P2": [0.091310, 0.137432],
            "P3": [0.032755, 0.161716], "P4": [-0.030635, 0.162131],
            "P5": [-0.089503, 0.138615], "P6": [-0.135160, 0.094640],
        }
        wanted = {
            "P1": "GREEN", "P2": "GREEN", "P3": "YELLOW",
            "P4": "YELLOW", "P5": "GREEN", "P6": "YELLOW",
        }
        centres = project_grids(grids, [640, 480], 1.05, 1.35, 0.45)
        image = bytearray([40, 70, 150] * 640 * 480)  # blue pickup-grid floor
        rgb = {"YELLOW": (245, 220, 20), "GREEN": (20, 220, 30)}
        for grid_id, centre in centres.items():
            u, v = (int(round(value)) for value in centre)
            for y in range(v - 6, v + 7):
                for x in range(u - 11, u + 12):
                    offset = (y * 640 + x) * 3
                    image[offset:offset + 3] = bytes(rgb[wanted[grid_id]])
        found = {
            grid_id: classify_roi(
                bytes(image), 640, 480, 640 * 3, "rgb8", centre, 16
            ).colour
            for grid_id, centre in centres.items()
        }
        self.assertEqual(found, wanted)


if __name__ == "__main__":
    unittest.main()
