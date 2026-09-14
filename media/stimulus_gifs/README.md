# Stimulus GIFs

Rendered previews of every stimulus in
[`experiments/ten_stimulus_panel.yaml`](../../experiments/ten_stimulus_panel.yaml),
committed so they are available without a working py5 / Java / display
setup — drop them straight into slides.

`_contact_sheet.png` is all ten as still frames in one image, for a single
overview slide.

**These are generated files.** They come from the real stimulus classes
running in a real py5 sketch driven by the experiment YAML, so they cannot
drift from what the assay actually displays — but that only holds if they are
re-rendered after a change. If you edit the panel, regenerate them:

```bash
python3 tools/render_stimulus_gifs.py \
    --experiment experiments/ten_stimulus_panel.yaml \
    --out-dir media/stimulus_gifs --seconds 4.0 --fps 20 --size 420
```

`--size 420` matters: at the large jitter's ±80 px amplitude the circle clips
the frame at the default canvas (diameter + 80).

Current settings: 420×420 px, 4 s at 20 fps, 220 px circle on a white
(`background_gray: 255`) ground.
