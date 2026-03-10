// Saltation Matrix Interactive Simulation
// f⁻ = [1, Vy] (left of guard, Vy controlled by slider)
// f⁺ = [1, 0]  (right of guard, fixed horizontal flow)
// Guard surface: x = 0

const SALTATION_AUTO_LOOP = true; // Set to false for single-run mode

new p5((p) => {

  // ── Tuning constants ────────────────────────────────────────────────────────
  const N         = 250;    // Number of particles
  const DT        = 0.025;  // Integration timestep (sim seconds)
  const POST_TIME = 1.5;    // Sim seconds to run after all particles cross
  const PPU       = 35;     // Pixels per simulation unit
  const GUARD     = 0;      // Guard surface x-position (sim coords)
  const PAUSE_MS  = 2000;   // Milliseconds to pause at start and end of each loop

  // ── Colours (RGB) ───────────────────────────────────────────────────────────
  const C_BG    = [255, 255, 255];
  const C_FIELD = [185, 185, 185];
  const C_GUARD = [50,  50,  50 ];
  const C_PART  = [0,   68,  136];   // Blue   #004488 (Paul Tol High Contrast) — particles
  const C_SALT  = [221, 170, 51 ];   // Yellow #DDAA33 (Paul Tol High Contrast) — saltation estimate
  const C_NAIVE = [187, 85,  102];   // Red    #BB5566 (Paul Tol High Contrast) — naive estimate

  // ── Canvas / layout state ───────────────────────────────────────────────────
  let W, H;          // Canvas dimensions (px)
  let OX, OY;        // Canvas pixel coords of sim origin (0, 0)
  let vySlider;      // p5 slider element

  // ── Simulation state ────────────────────────────────────────────────────────
  //   'idle'        — waiting for user to place distribution
  //   'drag'        — user is drawing the initial ellipse
  //   'pause_start' — 2 s pause showing initial distribution before running
  //   'run'         — simulation stepping forward
  //   'pause_end'   — 2 s pause showing final comparison
  //   'done'        — final frozen state (non-loop mode only)
  let mode       = 'idle';
  let dist       = null;   // {cx, cy, angle, s1, s2} — distribution params (sim coords)
  let dragOrigin = null;   // {x, y} canvas px — start of drag
  let parts      = [];     // [{x, y, crossed: bool}]
  let P0         = null;   // 2×2 initial covariance
  let allCrossed = false;
  let postTimer  = 0;      // Sim time elapsed after all particles crossed
  let pauseAt    = 0;      // millis() timestamp when current pause began

  // ── Math helpers ────────────────────────────────────────────────────────────

  const mmul = (A, B) => [
    [A[0][0]*B[0][0] + A[0][1]*B[1][0],  A[0][0]*B[0][1] + A[0][1]*B[1][1]],
    [A[1][0]*B[0][0] + A[1][1]*B[1][0],  A[1][0]*B[0][1] + A[1][1]*B[1][1]]
  ];

  const mT = A => [[A[0][0], A[1][0]], [A[0][1], A[1][1]]];

  // P = R diag(s1², s2²) Rᵀ
  const ellipseToCov = (angle, s1, s2) => {
    const c = Math.cos(angle), s = Math.sin(angle);
    return mmul([[c,-s],[s,c]], mmul([[s1*s1,0],[0,s2*s2]], [[c,s],[-s,c]]));
  };

  // Eigendecompose 2×2 symmetric → {l1, l2, angle}, l1 ≥ l2
  const eig2 = (P) => {
    const a = P[0][0], b = P[0][1], d = P[1][1];
    const disc  = Math.sqrt(Math.max(((a-d)/2)**2 + b*b, 0));
    const l1    = (a+d)/2 + disc;
    const l2    = (a+d)/2 - disc;
    const angle = (Math.abs(b) < 1e-9 && a >= d) ? 0 : Math.atan2(l1-a, b);
    return { l1: Math.max(l1,1e-9), l2: Math.max(l2,1e-9), angle };
  };

  // Sample from N(mean, P) via Cholesky
  const sampleN = (mean, P) => {
    const a=P[0][0], b=P[0][1], d=P[1][1];
    const l11 = Math.sqrt(Math.max(a, 1e-12));
    const l21 = b / l11;
    const l22 = Math.sqrt(Math.max(d - l21*l21, 1e-12));
    const z1 = p.randomGaussian(), z2 = p.randomGaussian();
    return { x: mean.x + l11*z1, y: mean.y + l21*z1 + l22*z2 };
  };

  // ── USER-DEFINED: saltation matrix ─────────────────────────────────────────
  // f⁻ = [1, Vy],  f⁺ = [1, 0],  guard normal n = [1, 0]
  // crossingState: {x, y} — mean state at the guard (available if needed)
  const computeSaltationMatrix = (Vy, crossingState) => {
    return [[1, 0], [-Vy, 1]];
  };

  const applySaltation = (P, Vy, crossingState) => {
    const S = computeSaltationMatrix(Vy, crossingState);
    return mmul(S, mmul(P, mT(S)));
  };

  // ── Coordinate transforms ───────────────────────────────────────────────────
  const s2c = (sx, sy) => ({ cx: OX + sx*PPU,   cy: OY - sy*PPU });
  const c2s = (cx, cy) => ({ x: (cx-OX)/PPU,    y: (OY-cy)/PPU  });

  // ── Draw: vector field ──────────────────────────────────────────────────────
  const drawField = (Vy) => {
    const spacing = 55, alen = 16;
    p.stroke(...C_FIELD);
    p.strokeWeight(1);
    p.fill(...C_FIELD);
    for (let cx = spacing/2; cx < W; cx += spacing) {
      for (let cy = spacing/2; cy < H; cy += spacing) {
        const { x } = c2s(cx, cy);
        const vx = 1, vy = x < GUARD ? Vy : 0;
        const mag = Math.sqrt(vx*vx + vy*vy);
        const dx =  (vx/mag)*alen;
        const dy = -(vy/mag)*alen;
        p.line(cx - dx*0.5, cy - dy*0.5, cx + dx*0.5, cy + dy*0.5);
        p.push();
        p.translate(cx + dx*0.5, cy + dy*0.5);
        p.rotate(Math.atan2(dy, dx));
        p.noStroke();
        p.triangle(0, 0, -6, -2.5, -6, 2.5);
        p.pop();
      }
    }
  };

  // ── Draw: guard surface ─────────────────────────────────────────────────────
  const drawGuard = () => {
    const { cx } = s2c(GUARD, 0);
    p.push();
    p.stroke(...C_GUARD);
    p.strokeWeight(1.5);
    p.drawingContext.setLineDash([8, 6]);
    p.line(cx, 0, cx, H);
    p.drawingContext.setLineDash([]);
    p.pop();
    p.noStroke();
    p.fill(...C_GUARD);
    p.textAlign(p.CENTER);
    p.textSize(W > 500 ? 22 : 14);
    p.text('guard', cx, W > 500 ? 22 : 16);
    p.textAlign(p.LEFT);
  };

  // ── Draw: 2σ covariance ellipse ─────────────────────────────────────────────
  const drawCovEllipse = (mean, cov, col, dashed = false) => {
    const { l1, l2, angle } = eig2(cov);
    const { cx: mx, cy: my } = s2c(mean.x, mean.y);
    p.push();
    p.translate(mx, my);
    p.rotate(-angle);
    p.noFill();
    p.stroke(...col);
    p.strokeWeight(2.5);
    if (dashed) p.drawingContext.setLineDash([8, 5]);
    p.ellipse(0, 0, 4*Math.sqrt(l1)*PPU, 4*Math.sqrt(l2)*PPU);
    p.drawingContext.setLineDash([]);
    p.pop();
  };

  // ── Draw: particle cloud ────────────────────────────────────────────────────
  const drawParts = () => {
    p.noStroke();
    p.fill(...C_PART, 140);
    for (const pt of parts) {
      const { cx, cy } = s2c(pt.x, pt.y);
      if (cx > -5 && cx < W+5 && cy > -5 && cy < H+5)
        p.circle(cx, cy, 5);
    }
  };

  // ── Draw: ellipse preview while dragging ────────────────────────────────────
  const drawPreview = () => {
    if (!dist) return;
    const { cx: mx, cy: my } = s2c(dist.cx, dist.cy);
    p.push();
    p.translate(mx, my);
    p.rotate(-dist.angle);
    p.noFill();
    p.stroke(...C_PART, 210);
    p.strokeWeight(2);
    p.drawingContext.setLineDash([5, 4]);
    p.ellipse(0, 0, 4*dist.s1*PPU, 4*dist.s2*PPU);
    p.drawingContext.setLineDash([]);
    p.fill(...C_PART);
    p.noStroke();
    p.circle(0, 0, 8);
    p.pop();
  };

  // ── Draw: legend ────────────────────────────────────────────────────────────
  const drawLegend = () => {
    const fontSize = W > 500 ? 22 : 12;
    const lineH    = fontSize + 10;                    // row-to-row spacing
    const totalH   = lineH * 3 + fontSize + 8;        // 4 rows
    const dotSize  = Math.round(fontSize * 0.6);      // particle dot diameter
    const lineLen  = Math.round(fontSize * 0.9);      // legend line length
    const sw       = W > 500 ? 3 : 2;                 // stroke weight
    const lx = 12, ly = H - totalH;
    const textX = lx + lineLen + 6;
    const textDY = Math.round(fontSize * 0.38);       // baseline offset from row centre
    p.textSize(fontSize);

    // Row 1: Particles — filled dot
    p.noStroke(); p.fill(...C_PART);
    p.circle(lx + lineLen/2, ly, dotSize);
    p.fill(60);
    p.text('Particles (ground truth)', textX, ly + textDY);

    // Row 2: Initial distribution — solid blue line
    p.stroke(...C_PART); p.strokeWeight(sw); p.noFill();
    p.line(lx, ly+lineH, lx+lineLen, ly+lineH);
    p.fill(60); p.noStroke();
    p.text('Initial distribution (\u03A3\u2080)', textX, ly+lineH+textDY);

    // Row 3: Saltation — solid yellow line
    p.stroke(...C_SALT); p.strokeWeight(sw); p.noFill();
    p.line(lx, ly+lineH*2, lx+lineLen, ly+lineH*2);
    p.fill(60); p.noStroke();
    p.text('Saltation (\u039E)', textX, ly+lineH*2+textDY);

    // Row 4: Jacobian of Reset — dashed red line
    p.stroke(...C_NAIVE); p.strokeWeight(sw); p.noFill();
    p.drawingContext.setLineDash([8, 5]);
    p.line(lx, ly+lineH*3, lx+lineLen, ly+lineH*3);
    p.drawingContext.setLineDash([]);
    p.fill(60); p.noStroke();
    p.text('Jacobian of Reset (D\u2093R)', textX, ly+lineH*3+textDY);
  };

  // ── Simulation init ─────────────────────────────────────────────────────────
  const initParts = () => {
    P0    = ellipseToCov(dist.angle, dist.s1, dist.s2);
    parts = Array.from({ length: N }, () => ({
      ...sampleN({ x: dist.cx, y: dist.cy }, P0),
      crossed: false
    }));
    allCrossed = false;
    postTimer  = 0;
  };

  // Advance one frame; returns true when post-crossing window has elapsed
  const step = (Vy) => {
    for (const pt of parts) {
      if (!pt.crossed) {
        pt.x += DT;
        pt.y += Vy * DT;
        if (pt.x >= GUARD) pt.crossed = true;
      } else {
        pt.x += DT;
      }
    }
    if (!allCrossed && parts.every(pt => pt.crossed)) {
      allCrossed = true;
      postTimer  = 0;
    }
    if (allCrossed) postTimer += DT;
    return allCrossed && postTimer >= POST_TIME;
  };

  // ── Helper: start a drag from any mode ─────────────────────────────────────
  const tryStartDrag = () => {
    const sim = c2s(p.mouseX, p.mouseY);
    if (sim.x < GUARD - 0.1 &&
        p.mouseX > 0 && p.mouseX < W &&
        p.mouseY > 0 && p.mouseY < H) {
      dist       = { cx: sim.x, cy: sim.y, angle: 0, s1: 0.3, s2: 0.12 };
      dragOrigin = { x: p.mouseX, y: p.mouseY };
      mode       = 'drag';
      return true;
    }
    return false;
  };

  // ── p5 setup ────────────────────────────────────────────────────────────────
  const getCanvasWidth = () => {
    const container = document.getElementById('saltation-canvas');
    const cw = container.getBoundingClientRect().width;
    return Math.min(700, cw > 10 ? cw : (window.innerWidth - 40));
  };

  p.setup = () => {
    W = getCanvasWidth();
    H = 450;
    p.createCanvas(W, H).parent('saltation-canvas');
    OX = W * 0.5;   // guard at horizontal centre
    OY = H * 0.5;
    p.textFont('sans-serif');

    vySlider = p.createSlider(-2.5, 2.5, 1.2, 0.05);
    vySlider.parent('saltation-vy');
    vySlider.style('width', '180px');
    vySlider.elt.style.accentColor = '#3366CC';
  };

  p.windowResized = () => {
    const newW = getCanvasWidth();
    if (Math.abs(newW - W) > 5) {
      W = newW;
      OX = W * 0.5;
      p.resizeCanvas(W, H);
    }
  };

  // ── p5 draw loop ────────────────────────────────────────────────────────────
  p.draw = () => {
    p.background(...C_BG);
    const Vy = vySlider ? vySlider.value() : 1.2;

    drawField(Vy);
    drawGuard();

    // ── idle: prompt ──────────────────────────────────────────────────────────
    if (mode === 'idle') {
      p.fill(140); p.noStroke();
      p.textAlign(p.CENTER); p.textSize(14);
      p.text('Click and drag on the left to place the initial distribution', W/2, H/2);
      p.textAlign(p.LEFT);
      return;
    }

    // ── drag: preview ellipse ─────────────────────────────────────────────────
    if (mode === 'drag') {
      drawPreview();
      return;
    }

    // ── pause_start: show initial distribution, wait PAUSE_MS ────────────────
    if (mode === 'pause_start') {
      drawParts();
      if (P0) drawCovEllipse({ x: dist.cx, y: dist.cy }, P0, C_PART, false);
      drawLegend();
      if (p.millis() - pauseAt >= PAUSE_MS) mode = 'run';
      return;
    }

    // ── run: step simulation, show ellipses after first crossing ──────────────
    if (mode === 'run') {
      drawParts();
      if (parts.some(pt => pt.crossed) && P0) {
        const n  = parts.length;
        const mx = parts.reduce((s, pt) => s + pt.x, 0) / n;
        const my = parts.reduce((s, pt) => s + pt.y, 0) / n;
        const mean = { x: mx, y: my };
        drawCovEllipse(mean, P0,                           C_NAIVE, true);
        drawCovEllipse(mean, applySaltation(P0, Vy, mean), C_SALT,  false);
      }
      drawLegend();
      const done = step(Vy);
      if (done) {
        pauseAt = p.millis();
        mode = 'pause_end';
      }
      return;
    }

    // ── pause_end / done: hold final state ────────────────────────────────────
    if (mode === 'pause_end' || mode === 'done') {
      drawParts();
      if (P0) {
        const n  = parts.length;
        const mx = parts.reduce((s, pt) => s + pt.x, 0) / n;
        const my = parts.reduce((s, pt) => s + pt.y, 0) / n;
        const mean = { x: mx, y: my };
        drawCovEllipse(mean, P0,                           C_NAIVE, true);
        drawCovEllipse(mean, applySaltation(P0, Vy, mean), C_SALT,  false);
      }
      drawLegend();
      if (mode === 'pause_end' && p.millis() - pauseAt >= PAUSE_MS) {
        if (SALTATION_AUTO_LOOP) {
          initParts();
          pauseAt = p.millis();
          mode = 'pause_start';
        } else {
          mode = 'done';
        }
      }
    }
  };

  // ── Mouse interaction ───────────────────────────────────────────────────────
  p.mousePressed = () => {
    if (mode === 'drag') return;  // ignore clicks mid-drag
    tryStartDrag();
  };

  p.mouseDragged = () => {
    if (mode !== 'drag' || !dragOrigin) return;
    const dx  = p.mouseX - dragOrigin.x;
    const dy  = -(p.mouseY - dragOrigin.y);
    const len = Math.sqrt(dx*dx + dy*dy);
    if (len > 8) {
      dist.angle = Math.atan2(dy, dx);
      dist.s1    = Math.max(0.08, len / PPU);
      dist.s2    = dist.s1 * 0.35;
    }
  };

  p.mouseReleased = () => {
    if (mode !== 'drag') return;
    if (dist && dist.s1 > 0.08) {
      initParts();
      pauseAt = p.millis();
      mode    = 'pause_start';
    } else {
      mode = 'idle';
    }
  };

  // ── Touch interaction (mirrors mouse handlers for mobile) ───────────────────
  // Return true (allow default) unless actively dragging — p5 registers these
  // at the document level, so returning false blocks all scroll/tap site-wide.
  p.touchStarted = () => {
    if (mode === 'drag') return false;
    const t = p.touches[0];
    if (!t) return true;
    const sim = c2s(t.x, t.y);
    if (sim.x < GUARD - 0.1 && t.x > 0 && t.x < W && t.y > 0 && t.y < H) {
      dist       = { cx: sim.x, cy: sim.y, angle: 0, s1: 0.3, s2: 0.12 };
      dragOrigin = { x: t.x, y: t.y };
      mode       = 'drag';
      return false;  // prevent scroll only when starting a canvas drag
    }
    return true;
  };

  p.touchMoved = () => {
    if (mode !== 'drag' || !dragOrigin) return true;  // allow scroll when not dragging
    const t = p.touches[0];
    if (!t) return true;
    const dx  = t.x - dragOrigin.x;
    const dy  = -(t.y - dragOrigin.y);
    const len = Math.sqrt(dx*dx + dy*dy);
    if (len > 8) {
      dist.angle = Math.atan2(dy, dx);
      dist.s1    = Math.max(0.08, len / PPU);
      dist.s2    = dist.s1 * 0.35;
    }
    return false;  // prevent scroll while actively dragging
  };

  p.touchEnded = () => {
    if (mode !== 'drag') return true;
    if (dist && dist.s1 > 0.08) {
      initParts();
      pauseAt = p.millis();
      mode    = 'pause_start';
    } else {
      mode = 'idle';
    }
    return false;
  };

});
