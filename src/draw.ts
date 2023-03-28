import p5 from 'p5';
import { Vec2 } from './utils';
import { Polygon } from './shape';
import { RigidBody, Contact, World, DistanceJoint } from './physics';

export class Draw {
  meterToPx: number;
  origin: Vec2;
  constructor(meterToPx: number, origin=Vec2.ZERO) {
    this.meterToPx = meterToPx;
    this.origin = origin;
  }

  posToPx(v: Vec2) {
    return this.origin.to(v).times(this.meterToPx);
  }

  drawPolygon(p: p5, poly: Polygon) {
    p.beginShape();
    for (const v of poly.vertices) {
      p.vertex(...this.posToPx(v).toTuple());
    }
    p.endShape(p.CLOSE);
  }

  drawBody(p: p5, body: RigidBody) {
    const shape = body.movedShape();
    if (!shape) { return; }

    p.colorMode(p.HSB, 1.0);
    let color = p.color(0.0, 0.2, 1.0);
    if (body.frozen) {
      color = p.color(0.6);
    }

    p.strokeWeight(3);
    p.stroke(color);
    p.noFill();
    this.drawPolygon(p, shape);
    p.circle(...this.posToPx(body.pos).toTuple(), 5);
  }

  drawContact(p: p5, contact: Contact) {
    const pos1 = this.posToPx(contact.body1.pos.add(contact.r1));
    const pos2 = this.posToPx(contact.body2.pos.add(contact.r2));

    p.colorMode(p.RGB, 255);
    let color = p.color(255, 0, 0);
    if (contact.isFrictionStatic) {
      color = p.color(0, 255, 0); // 静止摩擦なら緑
    }
    p.fill(color);
    p.noStroke();

    const r = 8;
    p.circle(...pos1.toTuple(), r);
    p.circle(...pos2.toTuple(), r);
  }

  drawJoint(p: p5, joint: DistanceJoint) {

  }

  drawWorld(p: p5, world: World) {
    for (const b of world.bodies) {
      this.drawBody(p, b);
    }
    for (const c of world.contacts) {
      this.drawContact(p, c);
    }
  }
}