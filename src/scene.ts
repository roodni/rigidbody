import p5 from 'p5';
import * as utils from './utils';
import { Vec2, Mat2 } from './utils';
import { PinJoint, RigidBody, World } from './physics';
import { Draw } from './draw';
import { Polygon, Collision, Circle, Shape } from './shape';
import config from './config';

export abstract class Scene {
  abstract init(): void;
  abstract update(p: p5): void;
  mousePressed(p: p5) {}
  mouseReleased(p: p5) {}
}

// 当たり判定確認
export class CollisionScene extends Scene {
  body1: RigidBody;
  angle: number;
  drawer: Draw;

  constructor() {
    super();
    this.body1 = RigidBody.fromShape(
      Polygon.regular(4, Vec2.c(250, 250), 120)
      // new Circle(Vec2.c(250, 250), 100)
    );

    this.angle = 0;
    this.drawer = new Draw(1.0);
  }

  init() {}
  update(p: p5) {
    if (p.keyIsPressed) {
      switch (p.key) {
        case 'a':
          this.angle -= Math.PI / 180;
          break;
        case 'd':
          this.angle += Math.PI / 180;
          break;
      }
    }

    const body2 = RigidBody.fromShape(
      Polygon.regular(4, Vec2.ZERO, 80)
      // new Circle(Vec2.ZERO, 80)
      // new Polygon([Vec2.ZERO, Vec2.c(120, -40), Vec2.c(40, 40)])
    );
    body2.pos = Vec2.c(p.mouseX, p.mouseY);
    body2.angle = this.angle;

    const shape1 = this.body1.movedShape()!;
    const shape2 = body2.movedShape()!;
    const col = shape1.collide(shape2);

    p.colorMode(p.RGB, 255);
    p.background(0);

    this.drawer.drawBody(p, this.body1);
    this.drawer.drawBody(p, body2);

    if (col) {
      p.colorMode(p.RGB, 255);
      p.stroke(255, 0, 0);
      p.noFill();
      for (const [p1, p2] of col.points) {
        p.line(p1.x, p1.y, p2.x, p2.y);
      }
      const [p1, p2] = col.points[0];
      p.circle(p1.x, p1.y, 5);
    }
  }

}


// 物理演算
abstract class WorldScene extends Scene {
  world: World;
  meterToPx: number;
  drawer: Draw;
  timestep = 1/60;

  gripRadius = 0.03;
  grabbing?: {body: RigidBody, loc: Vec2};

  startPos = Vec2.ZERO;

  constructor(canvasW: number, worldW: number) {
    super();
    this.world = new World();
    this.meterToPx = canvasW / worldW;
    this.drawer = new Draw(this.meterToPx);
  }

  updateWorld() {};

  update(p: p5) {
    this.world.gravity = Vec2.c(0, 9.8);
    if (p.keyIsPressed) {
      if (p.key === 'g') {
        this.world.gravity = Vec2.ZERO;
      }
    }

    this.updateWorld();
    this.world.step(this.timestep);

    p.colorMode(p.RGB, 255);
    p.background(0);
    this.drawer.drawWorld(p, this.world);

    if (p.mouseIsPressed && this.grabbing) {
      p.colorMode(p.RGB, 255);
      p.stroke(255);
      p.noFill();
      const body = this.grabbing.body;
      const p1 = this.drawer.pxToPos(Vec2.c(p.mouseX, p.mouseY));
      const p2 = body.pos.add(Mat2.rotate(body.angle).mulvec(this.grabbing.loc));
      const v2 = body.velAt(p2);
      const p21 = p2.to(p1);
      const dist = p21.norm();
      const dir = p21.safeNormalize(v2.safeNormalize(Vec2.ZERO));
      const spring = 9.8 * body.mass * dist / 0.1;
      body.addForce(p2, dir.times(spring).add(v2.times(-9.8 * body.mass)));
      p.circle(
        ...this.drawer.posToPx(p2).toTuple(),
        this.gripRadius * this.meterToPx * 2
      );
      p.line(p.mouseX, p.mouseY, ...this.drawer.posToPx(p2).toTuple());
    } else if (p.mouseIsPressed && !this.grabbing) {
      p.colorMode(p.RGB, 255);
      p.stroke(255);
      p.noFill();
      const p1 = this.startPos;
      const p2 = Vec2.c(p.mouseX, p.mouseY);
      if (config.shapeName === 'rectangle') {
        p.rect(p1.x, p1.y, p2.x -p1.x, p2.y -p1.y);
      } else if (config.shapeName === 'circle') {
        const dia = p1.to(p2).norm();
        const center = p1.add(p2).times(1/2);
        p.circle(center.x, center.y, dia);
      }
    }
  }

  mousePressed(p: p5): void {
    this.startPos = Vec2.c(p.mouseX, p.mouseY);
    // 当たり判定
    this.grabbing = undefined;
    const pos = this.drawer.pxToPos(this.startPos);
    const hitShape = new Circle(pos, this.gripRadius);
    for (let i = 0; i < this.world.shapes.length; i++) {
      const body = this.world.bodies[i];
      const shape = this.world.shapes[i];
      if (!shape || body.frozen) { continue; }
      if (hitShape.collide(shape)) {
        const rot = Mat2.rotate(-body.angle);
        const loc = rot.mulvec(body.pos.to(pos));
        this.grabbing = {body, loc};
        break;
      }
    }
  }
  mouseReleased(p: p5): void {
    if (this.grabbing) {
      return;
    }

    const p1 = this.drawer.pxToPos(this.startPos);
    const p2 = this.drawer.pxToPos(Vec2.c(p.mouseX, p.mouseY));
    const min = 0.05;

    let shape;
    if (config.shapeName === 'rectangle') {
      const w = Math.abs(p1.x -p2.x);
      const h = Math.abs(p1.y -p2.y);
      if (Math.min(w, h) < min) {
        return;
      }
      shape = Polygon.rect(...p1.toTuple(), ...p2.toTuple());
    } else if (config.shapeName === 'circle') {
      const center = p1.add(p2).times(1/2);
      const r = p1.to(p2).norm() / 2;
      if (r < min) {
        return;
      }
      shape = new Circle(center, r);
    }
    if (shape) {
      const body = RigidBody.fromShape(shape);
      if (config.freeze) {
        body.freeze();
      }
      this.world.addBody(body);
    }
  }
}


// 壁で囲まれたシーンのベース
export class BoxedWorldScene extends WorldScene {
  worldW: number;
  worldH: number;
  margin: number;

  constructor(canvasW: number, w=2.0, h=2.0, m=0.1) {
    super(canvasW, w);
    this.worldW = w;
    this.worldH = h;
    this.margin = m;
  }

  init() {
    const m = this.margin;
    const l = m;
    const r = this.worldW - m;
    const t = m;
    const b = this.worldH - m;

    [
      Polygon.wall(Vec2.c(l, b), Vec2.c(r, b), m),
      Polygon.wall(Vec2.c(r, b), Vec2.c(r, t), m),
      Polygon.wall(Vec2.c(r, t), Vec2.c(l, t), m),
      Polygon.wall(Vec2.c(l, t), Vec2.c(l, b), m),
    ].forEach((shape) => {
      const b = RigidBody.fromShape(shape);
      b.restitution = 0.9;
      b.friction = 1.0;
      b.freeze();
      this.world.addBody(b);
    });
  }
}

// bodyをランダムに生成する
function createBodies(num: number, lt: Vec2, rb: Vec2, [minR, maxR]=[0.1, 0.2]) {
  const bodies = [];
  for (let i = 0; i < num; i++) {
    const n = utils.randInt(2, 6);
    const radius = utils.rand(minR, maxR);
    const pos = Vec2.c(
      utils.rand(lt.x + radius, rb.x - radius),
      utils.rand(lt.y + radius, rb.y - radius)
    );
    let shape;
    if (n <= 2) {
      shape = new Circle(pos, radius);
    } else {
      const angle = utils.randInt(3, 6);
      shape = Polygon.regular(n, pos, radius, angle);
    }
    bodies.push(RigidBody.fromShape(shape));
  }
  return bodies;
}


export class BodiesScene extends BoxedWorldScene {
  constructor(canvasW: number) {
    super(canvasW);
  }

  init() {
    // this.timestep *= 0.5;
    super.init();

    const m = this.margin;
    const r = this.worldW - m;
    const b = this.worldH - m;

    createBodies(
      20, Vec2.c(m, m), Vec2.c(r, b), [0.1, 0.2]
    ).forEach((body) => {
      body.restitution = 0.2;
      body.friction = 0.5;
      this.world.addBody(body);
    });
  }

  updateWorld(): void {}
}


export class StackScene extends BoxedWorldScene {
  count = 0;
  constructor(canvasW: number) {
    super(canvasW);
  }

  init() {
    super.init();

    // const x1 = this.worldW / 4;
    // const x2 = this.worldW / 4 * 2;
    // const h = this.worldH / 10;

    // for (let i = 0; i < 8; i++) {
    //   const y2 = this.worldW - this.margin - (h * 1.2) * i;
    //   const rect = Polygon.rect(x1, y2 - h, x2, y2);
    //   this.world.addBody(
    //     RigidBody.fromShape(rect)
    //   );
    // }
  }

  updateWorld(): void {
    if (this.count % 60 == 0 && this.count < 60*8) {
      const x1 = this.worldW / 4;
      const x2 = this.worldW / 4 * 2;
      const h = this.worldH / 10;
      const rect = Polygon.rect(x1, this.margin, x2, this.margin + h);
      this.world.addBody(
        RigidBody.fromShape(rect)
      );
    }

    this.count++;
  }
}


export class PendulumScene extends BoxedWorldScene {
  constructor(canvasW: number) {
    super(canvasW);
  }

  init() {
    // super.init();
    const m = this.margin;
    const ww = this.worldW;
    const wh = this.worldH;

    let prev = this.world.ether;
    for (let i = 0; i < 6; i++) {
      const rw = 0.2;
      const rh = 0.05;
      const x1 = ww/2 + rw * i;
      const y1 = wh/3;

      const p = Polygon.rect(x1, y1, x1 +rw, y1 +rh);
      const body = RigidBody.fromShape(p);
      this.world.addBody(body);

      const pin = PinJoint.pin(prev, body, Vec2.c(x1, y1 +rh/2));
      this.world.addJoint(pin);

      prev = body;
    }
  }
}


// 床が抜けてループするシーンのベース
class LoopWorldScene extends WorldScene {
  worldW: number;
  worldH: number;
  margin: number;
  restitution = 1.0;
  friction = 1.0;

  constructor(canvasW: number, w=2.0, h=2.0, m=0.1) {
    super(canvasW, w);
    this.worldW = w;
    this.worldH = h;
    this.margin = m;
  }

  init() {
    const m = this.margin;
    const ww = this.worldW;
    const wh = this.worldH;
    const l = m;
    const r = ww - m;
    [
      Polygon.wall(Vec2.c(l, -wh), Vec2.c(l, wh*2), m),
      Polygon.wall(Vec2.c(r, wh*2), Vec2.c(r, -wh), m),
      Polygon.wall(Vec2.c(r, -wh), Vec2.c(l, -wh), m),
    ].forEach((shape) => {
      const b = RigidBody.fromShape(shape);
      b.restitution = this.restitution;
      b.friction = this.friction;
      b.freeze();
      this.world.addBody(b);
    });

  }

  updateWorld(): void {
    for (const b of this.world.bodies) {
      if (b.pos.y > this.worldH*1.5) {
        b.pos = Vec2.c(b.pos.x, b.pos.y - this.worldH*1.6);
      }
    }
  }
}

export class LoopScene1 extends LoopWorldScene {
  init() {
    super.init();

    const ww = this.worldW;
    const wh = this.worldH;
    const m = this.margin;
    const c = wh / 2;
    const r = ww - m;
    const b = wh - m;

    [
      Polygon.wall(Vec2.c(m, b), Vec2.c(ww/10*4, b), m),
      Polygon.wall(Vec2.c(ww/10*7, b), Vec2.c(r, b), m),

      Polygon.wall(Vec2.c(ww/4, c), Vec2.c(ww/4*3, c), m),
    ].forEach((shape) => {
      const b = RigidBody.fromShape(shape);
      b.restitution = this.restitution;
      b.friction = this.friction;
      b.freeze();
      this.world.addBody(b);
    });

    createBodies(20, Vec2.c(m, m), Vec2.c(r, b))
      .forEach((body) => {
        body.restitution = 0.5;
        body.friction = 0.3;
        this.world.addBody(body);
      });
  }
}

export class LoopScene2 extends LoopWorldScene {
  init() {
    super.init();

    const ww = this.worldW;
    const wh = this.worldH;
    const m = this.margin;

    const cx = ww / 2;
    const cy = wh / 2 - m*1.5;
    const r = ww/2 *0.8;
    const w = 0.05;

    const hori = RigidBody.fromShape(
      Polygon.rect(cx -r, cy -w, cx +r, cy +w)
    );
    const vert = RigidBody.fromShape(
      Polygon.rect(cx -w, cy -r, cx +w, cy +r)
    );
    this.world.addBody(hori);
    this.world.addBody(vert);

    [
      PinJoint.pin(this.world.ether, hori, Vec2.c(cx, cy)),
      PinJoint.pin(hori, vert, Vec2.c(cx - r, cy)),
      PinJoint.pin(hori, vert, Vec2.c(cx + r, cy)),
    ].forEach((joint) => {
      this.world.addJoint(joint);
    });

    [
      Polygon.wall(Vec2.c(0, wh -m*2), Vec2.c(ww/3, wh -m), m),
      Polygon.wall(Vec2.c(ww/3*2, wh -m), Vec2.c(ww, wh -m*2), m)
    ].forEach((shape) => {
      const body = RigidBody.fromShape(shape);
      body.freeze();
      this.world.addBody(body);
    });

    createBodies(15, Vec2.c(m, 0), Vec2.c(ww -m, wh), [0.1, 0.15])
      .forEach((body) => {
        body.restitution = 0.5;
        body.friction = 0.2;
        this.world.addBody(body);
      });
  }
}