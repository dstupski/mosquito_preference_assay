"""The marker types -- the visual "behaviours" a stimulus can have. Each is a
plain circle; what varies is what happens inside it.

Every Stimulus:

* is constructed as ``Cls(diameter_px, **params)`` where ``params`` are the
  keyword arguments below (all with defaults, so an experiment YAML only lists
  what it wants to change);
* is drawn once per frame as ``display(cx, cy, t)`` where ``t`` is seconds
  since *this instance* was placed on screen;
* implements ``describe()`` -> a plain JSON dict that *completely* specifies
  it (type + resolved params + a uuid), which is what the ROS node publishes.

To add a new marker behaviour: subclass Stimulus here, then register it in
STIMULUS_TYPES in stimulus_types.py. The experiment YAML can then compose
instances of it.
"""

import uuid

import py5


class Stimulus:
    type_name = "base"

    def __init__(self, diameter_px):
        self.uuid = str(uuid.uuid4())
        self.diameter_px = diameter_px

    def display(self, cx, cy, t):
        raise NotImplementedError

    def describe(self):
        """Complete, JSON-serializable description of this stimulus."""
        out = {
            "type": self.type_name,
            "uuid": self.uuid,
            "diameter_px": self.diameter_px,
        }
        out.update(self._params())
        return out

    def _params(self):
        """Subclass hook: the type-specific parameters, all JSON-safe."""
        return {}


# --- 1. Static dark circle: no motion at all. The baseline/control marker. ---
class StaticDarkStimulus(Stimulus):
    type_name = "static_dark"

    def __init__(self, diameter_px, fill_gray=20):
        super().__init__(diameter_px)
        self.fill_gray = fill_gray
        self._fill = py5.color(fill_gray)

    def display(self, cx, cy, t):
        py5.no_stroke()
        py5.fill(self._fill)
        py5.ellipse(cx, cy, self.diameter_px, self.diameter_px)

    def _params(self):
        return {"fill_gray": self.fill_gray}


# --- 2. Jitter: same dark circle, but its position wanders in a small ---
# --- random walk (Perlin noise, so the motion is smooth, not jumpy). ---
class JitterStimulus(Stimulus):
    type_name = "jitter"

    def __init__(self, diameter_px, fill_gray=20, amplitude_px=20, noise_speed=1.2,
                 seed_x=None, seed_y=None):
        super().__init__(diameter_px)
        self.fill_gray = fill_gray
        self._fill = py5.color(fill_gray)
        self.amplitude_px = amplitude_px
        self.noise_speed = noise_speed
        # Per-instance offsets into the (globally seeded, see assay.setup) noise
        # field so left/right copies don't wander in lockstep. Fixed at build
        # time -- pass explicit values from the trial RNG for reproducibility;
        # otherwise pick from py5's random stream.
        self.seed_x = py5.random(1000) if seed_x is None else seed_x
        self.seed_y = py5.random(1000) if seed_y is None else seed_y

    def display(self, cx, cy, t):
        dx = (py5.noise(self.seed_x + t * self.noise_speed) - 0.5) * 2 * self.amplitude_px
        dy = (py5.noise(self.seed_y + t * self.noise_speed) - 0.5) * 2 * self.amplitude_px
        py5.no_stroke()
        py5.fill(self._fill)
        py5.ellipse(cx + dx, cy + dy, self.diameter_px, self.diameter_px)

    def _params(self):
        return {
            "fill_gray": self.fill_gray,
            "amplitude_px": self.amplitude_px,
            "noise_speed": self.noise_speed,
            "seed_x": self.seed_x,
            "seed_y": self.seed_y,
        }


# --- 3. Moving grating: black/white stripes drifting across the circle ---
# --- at a constant velocity (a classic optomotor-style stimulus). ---
class MovingGratingStimulus(Stimulus):
    type_name = "moving_grating"

    def __init__(self, diameter_px, period_px=24, speed_px_per_sec=40, angle_deg=0.0,
                 color_a_gray=240, color_b_gray=20):
        super().__init__(diameter_px)
        self.period_px = period_px
        self.speed_px_per_sec = speed_px_per_sec
        self.angle_deg = angle_deg
        self.color_a_gray = color_a_gray
        self.color_b_gray = color_b_gray
        self._color_a = py5.color(color_a_gray)
        self._color_b = py5.color(color_b_gray)
        self._buf = None
        self._buf_diameter = None

    def display(self, cx, cy, t):
        d = round(self.diameter_px)
        if self._buf is None or self._buf_diameter != d:
            self._buf = py5.create_graphics(d, d)
            self._buf_diameter = d

        offset = (t * self.speed_px_per_sec) % self.period_px
        span = d * 0.75  # covers the buffer corner-to-corner after rotation

        buf = self._buf
        buf.begin_draw()
        buf.background(self._color_a)
        buf.push_matrix()
        buf.translate(d / 2, d / 2)
        buf.rotate(py5.radians(self.angle_deg))
        buf.no_stroke()
        buf.fill(self._color_b)
        x = -span - self.period_px
        while x < span:
            buf.rect(x + offset, -span, self.period_px / 2, span * 2)
            x += self.period_px
        buf.pop_matrix()
        buf.end_draw()

        buf.mask(circle_mask(d))
        py5.image_mode(py5.CENTER)
        py5.image(buf, cx, cy)

    def _params(self):
        return {
            "period_px": self.period_px,
            "speed_px_per_sec": self.speed_px_per_sec,
            "angle_deg": self.angle_deg,
            "color_a_gray": self.color_a_gray,
            "color_b_gray": self.color_b_gray,
        }


# --- 4. Telescope: concentric rings that appear to continuously expand ---
# --- outward from the center -- a hypnotic "tunnel" effect. ---
class TelescopeStimulus(Stimulus):
    type_name = "telescope"

    def __init__(self, diameter_px, ring_spacing_px=18, speed_px_per_sec=50,
                 color_a_gray=240, color_b_gray=20):
        super().__init__(diameter_px)
        self.ring_spacing_px = ring_spacing_px
        self.speed_px_per_sec = speed_px_per_sec
        self.color_a_gray = color_a_gray
        self.color_b_gray = color_b_gray
        self._color_a = py5.color(color_a_gray)
        self._color_b = py5.color(color_b_gray)

    def display(self, cx, cy, t):
        max_r = self.diameter_px / 2
        # Painting filled circles from largest to smallest, alternating color,
        # leaves a ring visible at each boundary -- no clipping mask needed
        # since every shape drawn is itself already circular.
        cycle = self.ring_spacing_px * 2
        phase = (t * self.speed_px_per_sec) % cycle

        py5.no_stroke()
        use_color_a = True
        r = max_r + phase
        while r > 0:
            py5.fill(self._color_a if use_color_a else self._color_b)
            clamped_r = min(r, max_r)
            py5.ellipse(cx, cy, clamped_r * 2, clamped_r * 2)
            use_color_a = not use_color_a
            r -= self.ring_spacing_px

    def _params(self):
        return {
            "ring_spacing_px": self.ring_spacing_px,
            "speed_px_per_sec": self.speed_px_per_sec,
            "color_a_gray": self.color_a_gray,
            "color_b_gray": self.color_b_gray,
        }


# Cached circular alpha masks, one per diameter actually used. Needed by any
# stimulus that draws a rectangular pattern (e.g. stripes) into an offscreen
# buffer and then wants only the circular part of it visible.
_mask_cache = {}


def circle_mask(d):
    cached = _mask_cache.get(d)
    if cached is not None:
        return cached

    m = py5.create_image(d, d, py5.ARGB)
    m.load_pixels()
    r = d / 2
    for y in range(d):
        for x in range(d):
            dist_from_center = py5.dist(x + 0.5, y + 0.5, r, r)
            v = 255 if dist_from_center <= r else 0
            m.pixels[y * d + x] = py5.color(v)  # mask() reads this as a grayscale alpha map
    m.update_pixels()
    _mask_cache[d] = m
    return m
