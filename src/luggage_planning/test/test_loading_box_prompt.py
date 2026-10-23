from luggage_planning.hardware_loading_cycle import loading_box_prompt
import unittest


class LoadingBoxPromptTests(unittest.TestCase):
    def test_prompt_follows_greedy_order_not_input_order(self):
        profile = dict(cycles=[
            dict(box_label="black", expected_demo_input_dims_m=[.67, .43, .26]),
            dict(box_label="red", expected_demo_input_dims_m=[.59, .37, .24]),
            dict(box_label="blue", expected_demo_input_dims_m=[.55, .36, .23]),
            dict(box_label="grey", expected_demo_input_dims_m=[.55, .36, .23])])
        for number, color in enumerate(["black", "red", "blue", "grey"], 1):
            prompt = loading_box_prompt(profile, dict(next_box=number))
            self.assertIn(color, prompt)
            self.assertIn("%d/4" % number, prompt)
        self.assertIn("67 × 43 × 26 cm", loading_box_prompt(profile, dict(next_box=1)))
        self.assertEqual(loading_box_prompt(profile, dict(next_box=None)), "全部完成")
