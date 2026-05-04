from python_service.bow_pressure import BowPressure


def main() -> None:
    bow_pressure = BowPressure(
        model_path="recordings/calibration1/ads1263_stream_mlp_model.json",
        serial_port="/dev/ttyACM0",
        buffer_size=100,
        calibration="calibrations/bow2_crap_params.json",
        default_contact_fraction=0.5,
        force_mv_source="predicted",
    )
    with bow_pressure:
        while True:
            r = bow_pressure.poll()
            if r.force_N is not None:
                print(
                    f"ts_us={r.arduino_us}  F={r.force_N:.4f} N  "
                    f"mV_pred={r.mv_predicted:.4f}  mV_ema={r.mv_ema:.4f}"
                )
            else:
                print(
                    f"ts_us={r.arduino_us}  F=(n/a)  "
                    f"mV_pred={r.mv_predicted:.4f}  mV_ema={r.mv_ema:.4f}"
                )


if __name__ == "__main__":
    main()
