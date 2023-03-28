import p5 from 'p5';
import * as utils from './utils';
import { Vec2 } from './utils';
import { DistanceJoint, RigidBody, World } from './physics';
import { Draw } from './draw';
import { Polygon } from './shape';

export abstract class Scene {
  abstract init(p: p5): void;
  abstract update(p: p5): void;
}

abstract class WorldScene extends Scene {
  world: World;
  drawer: Draw;
  timestep = 1/60;

  constructor(canvasW: number, worldW: number) {
    super();
    this.world = new World();
    this.drawer = new Draw(canvasW / worldW);
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
  }
}


// 壁で囲まれたシーンのベース
abstract class BoxedWorldScene extends WorldScene {
  worldW: number;
  worldH: number;
  margin: number;

  constructor(canvasW: number, w=2.0, h=2.0, m=0.1) {
    super(canvasW, w);
    this.worldW = w;
    this.worldH = h;
    this.margin = m;
  }

  init(p: p5) {
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
      const b = RigidBody.from_polygon(shape);
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
    const radius = utils.rand(minR, maxR);
    const poly = Polygon.regular(
      utils.randInt(3, 6),
      Vec2.c(
        utils.rand(lt.x + radius, rb.x - radius),
        utils.rand(lt.y + radius, rb.y - radius)
      ),
      radius,
      utils.rand(0, Math.PI * 2)
    );
    bodies.push(RigidBody.from_polygon(poly));
  }
  return bodies;
}


export class BodiesSchene extends BoxedWorldScene {
  constructor(canvasW: number) {
    super(canvasW);
  }

  init(p: p5) {
    // this.timestep *= 0.5;
    super.init(p);

    const m = this.margin;
    const r = this.worldW - m;
    const b = this.worldH - m;

    createBodies(
      15, Vec2.c(m, m), Vec2.c(r, b), [0.1, 0.25]
    ).forEach((body) => {
      body.restitution = 0.2;
      body.friction = 0.5;
      this.world.addBody(body);
    });
  }

  updateWorld(): void {}
}

export class PendulumScene extends BoxedWorldScene {
  constructor(canvasW: number) {
    super(canvasW);
  }

  init(p: p5) {
    // super.init(p);
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
      const body = RigidBody.from_polygon(p);
      this.world.addBody(body);

      const pin = DistanceJoint.pin(prev, body, Vec2.c(x1, y1 +rh/2));
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

  init(p: p5) {
    const m = this.margin;
    const ww = this.worldW;
    const wh = this.worldH;
    const l = m;
    const r = ww - m;
    [
      Polygon.wall(Vec2.c(l, -wh), Vec2.c(l, wh*2), m),
      Polygon.wall(Vec2.c(r, wh*2), Vec2.c(r, -wh), m),
    ].forEach((shape) => {
      const b = RigidBody.from_polygon(shape);
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
  init(p: p5) {
    super.init(p);

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
      const b = RigidBody.from_polygon(shape);
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
  init(p: p5) {
    super.init(p);

    const ww = this.worldW;
    const wh = this.worldH;
    const m = this.margin;

    const cx = ww / 2;
    const cy = wh / 2 - m*1.5;
    const r = ww/2 *0.8;
    const w = 0.05;

    const hori = RigidBody.from_polygon(
      Polygon.rect(cx -r, cy -w, cx +r, cy +w)
    );
    const vert = RigidBody.from_polygon(
      Polygon.rect(cx -w, cy -r, cx +w, cy +r)
    );
    this.world.addBody(hori);
    this.world.addBody(vert);

    [
      DistanceJoint.pin(this.world.ether, hori, Vec2.c(cx, cy)),
      DistanceJoint.pin(hori, vert, Vec2.c(cx - r, cy)),
      DistanceJoint.pin(hori, vert, Vec2.c(cx + r, cy)),
    ].forEach((joint) => {
      this.world.addJoint(joint);
    });

    [
      Polygon.wall(Vec2.c(0, wh -m*2), Vec2.c(ww/3, wh -m), m),
      Polygon.wall(Vec2.c(ww/3*2, wh -m), Vec2.c(ww, wh -m*2), m)
    ].forEach((shape) => {
      const body = RigidBody.from_polygon(shape);
      body.freeze();
      this.world.addBody(body);
    });

    createBodies(15, Vec2.c(m, 0), Vec2.c(ww -m, wh), [0.1, 0.15])
      .forEach((body) => {
        body.restitution = 0.5;
        body.friction = 0.0;
        this.world.addBody(body);
      });
  }
}