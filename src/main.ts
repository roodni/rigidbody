import p5 from 'p5';
import { Vec2 } from './utils';
import { Polygon } from './shape';
import { RigidBody, World } from './physics';
import { Draw } from './draw';

const sketch = (p: p5) => {
  const canvasW = 500;
  const canvasH = 500;
  const worldW = 2.0;
  const worldH = worldW * canvasH / canvasW;

  const world = new World();
  const drawer = new Draw(p, canvasW / worldW);

  p.setup = () => {
    p.createCanvas(canvasW, canvasH);

    // 壁の追加
    const m = 0.1;
    const l = m;
    const r = worldW - m;
    const t = -worldW;
    const b = worldH * 2;
    [
      Polygon.wall(Vec2.c(worldW/4, worldH/2), Vec2.c(worldW/4*3, worldH/2), m),
      Polygon.wall(Vec2.c(0, worldH -m), Vec2.c(worldW/5*2, worldH -m), m),
      Polygon.wall(Vec2.c(worldW/5*3, worldH -m), Vec2.c(worldW, worldH -m), m),

      // Polygon.wall(Vec2.c(r, m), Vec2.c(l, m), m),
      // Polygon.wall(Vec2.c(l, worldH - m), Vec2.c(r, worldH - m), m),

      Polygon.wall(Vec2.c(r, b), Vec2.c(r, t), m),
      Polygon.wall(Vec2.c(l, t), Vec2.c(l, b), m)
    ].forEach((shape, i) => {
      const b = RigidBody.from_polygon(shape);
      b.freeze();
      world.addBody(b);
    });

    // なんか追加
    function randInt(a: number, b: number) {
      const n = b - a + 1;
      const r = Math.floor(Math.random() * n);
      return a + r;
    }
    function rand(a: number, b: number) {
      return a + Math.random() * (b - a);
    }

    for (let i = 0; i < 10; i++) {
      const radius = rand(0.1, 0.15);
      const poly = Polygon.regular(
        randInt(3, 6),
        Vec2.c(
          rand(radius, worldW - radius),
          rand(radius, worldH - radius)
        ),
        radius,
        rand(0, Math.PI * 2)
      );
      const body = RigidBody.from_polygon(poly);
      world.addBody(body);
    }
  };

  p.draw = () => {
    world.gravity = Vec2.c(0, 9.8);
    if (p.keyIsPressed) {
      if (p.key === 'g') {
        world.gravity = Vec2.ZERO;
      }
    }

    const loop = 1;
    for (let i = 0; i < loop; i++) {
      for (const b of world.bodies) {
        if (b.pos.y > worldH * 1.5) {
          b.pos = Vec2.c(b.pos.x, b.pos.y - worldH * 1.6)
        }
      }
      world.step(1/60 / loop);
    }

    p.colorMode(p.RGB, 255);
    p.background(0);
    drawer.drawWorld(world);
  };
};

let myp5 = new p5(sketch);