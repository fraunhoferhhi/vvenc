"""The retired DOT_PROXY pilot cannot be used for scientific training.

Run the integrated, versioned pipeline instead:
  python -m scripts.rl_rc_dot.session --help
For a short pilot use --total-timesteps 256 --rollout-steps 64
--validation-videos 2 --validation-rates 50,120 --validation-frames 64.
Keep --frames 0 if you intend to resume on complete training clips.
"""
if __name__=="__main__":
    raise SystemExit(__doc__)
