#!/usr/bin/env python3
"""
main.py – Physical Challenge–Response System (PCS) entry point.

Quick start (simulation, no hardware):
    python main.py --mock

With real hardware (edit ports first in config.py or use flags):
    python main.py \\
        --leader-port /dev/ttyACM0 \\
        --prover-port  /dev/ttyACM1 \\
        --verifier-port /dev/ttyACM2 \\
        --verifier-camera 0

Repeatedly run rounds:
    python main.py --mock --rounds 5
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

from config import (
    DEFAULT_CONFIG,
    MOCK_CONFIG,
    TRUST_THRESHOLD,
    ArmConfig,
    PCSConfig,
)
from pcs.protocol import PhysicalChallengeResponseSystem


# ── Logging setup ─────────────────────────────────────────────────────────────

def setup_logging(verbose: bool = False) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s  %(levelname)-7s  %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )
    # Quieten noisy third-party loggers
    for lib in ("PIL", "matplotlib", "urllib3", "asyncio"):
        logging.getLogger(lib).setLevel(logging.WARNING)


# ── Argument parsing ──────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Physical Challenge–Response System for SO-101 robot arms",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    p.add_argument(
        "--mock", action="store_true",
        help="Run in simulation mode (no hardware required).",
    )
    p.add_argument(
        "--rounds", type=int, default=1, metavar="N",
        help="Number of PCS rounds to run (default: 1).",
    )
    p.add_argument(
        "--threshold", type=float, default=TRUST_THRESHOLD, metavar="T",
        help=f"Trust score threshold for handoff (default: {TRUST_THRESHOLD}).",
    )
    p.add_argument(
        "--leader-port", default=DEFAULT_CONFIG.leader.port, metavar="PORT",
        help="Serial port for the Leader arm.",
    )
    p.add_argument(
        "--prover-port", default=DEFAULT_CONFIG.prover.port, metavar="PORT",
        help="Serial port for the Prover arm.",
    )
    p.add_argument(
        "--verifier-port", default=DEFAULT_CONFIG.verifier.port, metavar="PORT",
        help="Serial port for the Verifier arm.",
    )
    p.add_argument(
        "--verifier-camera", type=int, default=0, metavar="IDX",
        help="OpenCV camera index for the Verifier wrist cam (default: 0).",
    )
    p.add_argument(
        "--no-verifier-camera", action="store_true",
        help="Disable wrist camera on the Verifier arm.",
    )
    p.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable DEBUG-level logging.",
    )
    return p.parse_args()


# ── Config builder ────────────────────────────────────────────────────────────

def build_config(args: argparse.Namespace) -> PCSConfig:
    if args.mock:
        cfg = MOCK_CONFIG
        cfg.trust_threshold = args.threshold
        return cfg

    return PCSConfig(
        leader=ArmConfig(
            role="leader",
            port=args.leader_port,
            mock=False,
        ),
        prover=ArmConfig(
            role="prover",
            port=args.prover_port,
            has_camera=False,
            mock=False,
        ),
        verifier=ArmConfig(
            role="verifier",
            port=args.verifier_port,
            has_camera=not args.no_verifier_camera,
            camera_index=args.verifier_camera,
            mock=False,
        ),
        trust_threshold=args.threshold,
    )


# ── Signal handler ────────────────────────────────────────────────────────────

_pcs_instance: PhysicalChallengeResponseSystem | None = None


def _handle_sigint(sig, frame):
    print("\n[main] Interrupted – shutting down gracefully…")
    if _pcs_instance is not None:
        _pcs_instance.stop()
    sys.exit(0)


# ── Summary printer ───────────────────────────────────────────────────────────

def print_summary(results) -> None:
    print("\n" + "═" * 50)
    print(f"  PCS SESSION SUMMARY  ({len(results)} round{'s' if len(results) != 1 else ''})")
    print("═" * 50)
    grants = sum(1 for r in results if r.trust.granted)
    denies = len(results) - grants
    print(f"  Granted : {grants}")
    print(f"  Denied  : {denies}")
    print()
    for r in results:
        verdict = "GRANTED ✓" if r.trust.granted else "DENIED  ✗"
        ctype = r.challenge.challenge_type.name
        print(
            f"  [{r.round_id}]  {ctype:<20}  "
            f"score={r.trust.composite:.3f}  {verdict}"
        )
    print("═" * 50 + "\n")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    args = parse_args()
    setup_logging(args.verbose)

    log = __import__("logging").getLogger("main")
    cfg = build_config(args)

    mode = "SIMULATION" if args.mock else "HARDWARE"
    log.info("Starting PCS  mode=%s  rounds=%d  threshold=%.2f",
             mode, args.rounds, cfg.trust_threshold)
    if not args.mock:
        log.info("  leader  → %s", cfg.leader.port)
        log.info("  prover  → %s", cfg.prover.port)
        log.info("  verifier→ %s  camera=%s",
                 cfg.verifier.port,
                 cfg.verifier.camera_index if cfg.verifier.has_camera else "off")

    global _pcs_instance
    pcs = PhysicalChallengeResponseSystem(cfg)
    _pcs_instance = pcs

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        pcs.start()
        results = []
        for i in range(args.rounds):
            round_id = f"round-{i + 1:03d}"
            log.info("")
            result = pcs.run_round(round_id=round_id)
            results.append(result)

            # Brief pause between rounds
            if i < args.rounds - 1:
                log.info("Pause before next round…")
                time.sleep(2.0)

        print_summary(results)
        return 0

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received.")
        return 130

    except Exception as exc:
        log.exception("Fatal error: %s", exc)
        return 1

    finally:
        pcs.stop()


if __name__ == "__main__":
    sys.exit(main())
