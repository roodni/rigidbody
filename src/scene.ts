import p5 from 'p5';
import * as utils from './utils';
import { Vec2 } from './utils';
import { RigidBody, World } from './physics';
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


// 壁で囲まれたシーン
abstract class BoxedWorldScene extends WorldScene {
  worldW: number;
  worldH: number;
  margin: number;

  constructor(canvasW: number, w: number, h: number, m: number) {
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


export class MySchene extends BoxedWorldScene {
  constructor(canvasW: number) {
    super(canvasW, 2.0, 2.0, 0.1);
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

// 床が抜けてループするシーン
export class LoopWorldScene extends WorldScene {
  worldW: number;
  worldH: number;

  constructor(canvasW: number) {
    const w = 2.0;
    super(canvasW, w);
    this.worldW = w;
    this.worldH = w;
  }

  init(p: p5) {
    this.timestep *= 0.75;

    const m = 0.1;
    const ww = this.worldW;
    const wh = this.worldH;

    const l = m;
    const r = ww - m;
    const c = wh / 2;
    const b = this.worldH - m;

    [
      Polygon.wall(Vec2.c(l, -wh), Vec2.c(l, wh*2), m),
      Polygon.wall(Vec2.c(r, wh*2), Vec2.c(r, -wh), m),

      Polygon.wall(Vec2.c(l, b), Vec2.c(ww/10*4, b), m),
      Polygon.wall(Vec2.c(ww/10*7, b), Vec2.c(r, b), m),

      Polygon.wall(Vec2.c(ww/4, c), Vec2.c(ww/4*3, c), m),
    ].forEach((shape) => {
      const b = RigidBody.from_polygon(shape);
      b.restitution = 1.0;
      b.friction = 1.0;
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

  updateWorld(): void {
    for (const b of this.world.bodies) {
      if (b.pos.y > this.worldH*1.5) {
        b.pos = Vec2.c(b.pos.x, b.pos.y - this.worldH*1.6);
      }
    }
  }

}