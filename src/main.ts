import p5 from 'p5';
import {Vec2} from './utils';
import {Polygon, Collision} from './shape';
import {RigidBody, World} from './physics';

const sketch = (p: p5) => {
  const canvasW = 500;
  const canvasH = 500;
  const worldW = 2.0;
  const worldH = worldW * canvasH / canvasW;

  function vec_to_cvs(v: Vec2): [number, number] {
    return [
      v.x / worldW * canvasW,
      v.y / worldH * canvasH
    ];
  };

  function drawPolygon(poly: Polygon) {
    p.beginShape();
    for (const v of poly.vertices) {
      p.vertex(...vec_to_cvs(v))
    }
    p.endShape(p.CLOSE);
  }
  function drawBody(body: RigidBody) {
    p.strokeWeight(3);
    p.stroke(body.frozen ? [128, 128, 255] : [255]);
    p.noFill();
    drawPolygon(body.movedShape());
    p.circle(...vec_to_cvs(body.pos), 5);
  }

  const world = new World();

  p.setup = () => {
    p.createCanvas(canvasW, canvasH);

    // 壁の追加
    const m = 0.1;
    const lb = Vec2.c(m, worldH - m);
    const rb = Vec2.c(worldW - m, worldH - m);
    const rt = Vec2.c(worldW - m, m);
    const lt = Vec2.c(m, m);
    [
      Polygon.wall(lb, rb, m),
      Polygon.wall(rb, rt, m),
      Polygon.wall(rt, lt, m),
      Polygon.wall(lt, lb, m)
    ].forEach((shape) => {
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

    for (let i = 0; i < 15; i++) {
      const radius = rand(0.1, 0.2);
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

    p.background(0);
    const loop = 20;
    for (let i = 0; i < loop; i++) {
      world.step(1/60 / loop);
    }

    for (const b of world.bodies) {
      drawBody(b);
    }
  };
};

let myp5 = new p5(sketch);