import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from fit_calibration import fit_model, load_samples, main, predict, equation


class FitTests(unittest.TestCase):
    def test_bilinear_coefficients_and_prediction(self):
        samples = [(x, y, 2.5*x*y - 0.7*x + 1.2*y - 3)
                   for x in (1, 2, 5) for y in (0.05, 0.45, 0.95)]
        result = fit_model(samples, "bilinear")
        for key, value in dict(a=2.5, b=-0.7, c=1.2, d=-3).items():
            self.assertAlmostEqual(result[key], value)
        self.assertAlmostEqual(result["rmse"], 0)
        self.assertAlmostEqual(predict(result, 3, 0.6), 2.5*3*0.6 - 0.7*3 + 1.2*0.6 - 3)
        self.assertIn("-0.7 x", equation(result))
        self.assertIn("+1.2 y", equation(result))
        self.assertEqual(result["model"], "z = a*x*y + b*x + c*y + d")

    def test_bilinear_rejects_unidentifiable_data(self):
        cases = [[], [(1, .1, 2)] * 3,
                 [(x, .5, x) for x in (1, 2, 3, 4)],
                 [(x, x / 10, x) for x in (1, 2, 3, 4)]]
        for samples in cases:
            with self.subTest(samples=samples), self.assertRaises(ValueError):
                fit_model(samples, "bilinear")

    def test_bilinear_cli_uses_separate_outputs_and_filters_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.csv"
            rows = ["force_N,distance_fraction,fullbridge_mv", "0,0.5,999999"]
            rows += [f"{x},{y},{2*x*y+3*x-4*y+5}"
                     for x in (1, 2, 3) for y in (.1, .5, .9)]
            path.write_text("\n".join(rows) + "\n")
            simple_output = path.with_name("calibration-fit.json")
            simple_output.write_text("keep existing simple fit")
            with patch("sys.stdout"), patch("fit_calibration.plot_fit") as plot:
                self.assertEqual(main([str(path), "--model", "bilinear", "--show"]), 0)
            result = json.loads(path.with_name("calibration-fit-bilinear.json").read_text())
            self.assertEqual(result["zero_force_excluded"], 1)
            self.assertEqual(result["samples_used"], 9)
            for key, expected in dict(a=2, b=3, c=-4, d=5).items():
                self.assertAlmostEqual(result[key], expected)
            self.assertEqual(result["units"]["b"], "mV/N")
            self.assertEqual(result["units"]["d"], "mV")
            self.assertEqual(plot.call_args.args[1]["model_type"], "bilinear")
            self.assertEqual(plot.call_args.args[2], path.with_name("calibration-fit-bilinear.png"))
            self.assertTrue(plot.call_args.kwargs["show"])
            self.assertEqual(simple_output.read_text(), "keep existing simple fit")

    def test_known_model(self):
        samples = [(x, y, 2.5 * x * y - 1.3)
                   for x in (1, 2, 5) for y in (0.05, 0.45, 0.95)]
        result = fit_model(samples)
        self.assertAlmostEqual(result["a"], 2.5)
        self.assertAlmostEqual(result["c"], -1.3)
        self.assertAlmostEqual(result["rmse"], 0)
        self.assertAlmostEqual(result["r_squared"], 1)

    def test_noisy_model_metrics(self):
        # Predictions are 3,5,7; residuals 1,-2,1 are orthogonal to 1 and t.
        result = fit_model([(2, 0.5, 4), (4, 0.5, 3), (6, 0.5, 8)])
        self.assertAlmostEqual(result["a"], 2)
        self.assertAlmostEqual(result["c"], 1)
        self.assertAlmostEqual(result["rmse"], 2 ** 0.5)
        self.assertAlmostEqual(result["mae"], 4 / 3)
        self.assertEqual(result["max_abs_error"], 2)
        self.assertAlmostEqual(result["r_squared"], 4 / 7)

    def test_filter_and_units(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.csv"
            path.write_text("force_N,distance_fraction,fullbridge_mv,fullbridge_raw24\n"
                            "0,0.1,9999,9999\n-0.0,0.2,-9999,-9999\n"
                            "0.5,0.2,1,100\n-2,0.3,2,200\n"
                            "nan,0.5,1,100\n2,0.5,inf,100\n2,2,1,100\n")
            samples, counts = load_samples(path, "fullbridge_mv")
            self.assertEqual(samples, [(0.5, 0.2, 1), (-2, 0.3, 2)])
            self.assertEqual(counts, dict(rows_read=7, zero_force_excluded=2, invalid_rows_excluded=3))
            raw, _ = load_samples(path, "fullbridge_raw24")
            self.assertEqual(raw[0][2], 100)

    def test_insufficient_or_constant_product(self):
        for samples in ([], [(1, 0.5, 2)], [(1, 0.5, 2), (2, 0.25, 3)]):
            with self.subTest(samples=samples), self.assertRaises(ValueError):
                fit_model(samples)

    def test_constant_reading(self):
        result = fit_model([(1, 0.1, 3), (2, 0.3, 3)])
        self.assertEqual(result["a"], 0)
        self.assertEqual(result["c"], 3)
        self.assertEqual(result["rmse"], 0)
        self.assertIsNone(result["r_squared"])

    def test_cli_zero_outlier_does_not_bias_fit_and_metadata_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.csv"
            path.write_text("force_N,distance_fraction,fullbridge_mv\n"
                            "0,0.5,100000\n2,0.5,3\n4,0.5,5\n6,0.5,7\n")
            metadata = path.with_suffix(".json")
            metadata.write_text('{"status":"complete"}')
            with patch("sys.stdout"), patch("fit_calibration.plot_fit") as plot:
                self.assertEqual(main([str(path)]), 0)
            self.assertEqual(plot.call_args.args[0], [(2, 0.5, 3), (4, 0.5, 5), (6, 0.5, 7)])
            self.assertEqual(plot.call_args.args[2], path.with_name("calibration-fit.png"))
            result = json.loads(path.with_name("calibration-fit.json").read_text())
            self.assertEqual(result["a"], 2)
            self.assertEqual(result["c"], 1)
            self.assertEqual(result["zero_force_excluded"], 1)
            self.assertEqual(result["samples_used"], 3)
            with patch("sys.stderr"), self.assertRaises(SystemExit):
                main([str(path), "--output", str(metadata)])
            self.assertEqual(metadata.read_text(), '{"status":"complete"}')


if __name__ == "__main__":
    unittest.main()
