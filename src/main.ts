import p5 from 'p5';
import {Vec2, Polygon, Collision} from './shape';

const sketch = (p: p5) => {
  function drawPolygon(poly: Polygon) {
    p.beginShape();
    for (const v of poly.vs) {
      p.vertex(v.x, v.y);
    }
    p.endShape(p.CLOSE);
  }

  const poly1 = Polygon.regular(
    4, new Vec2(200, 180), 150
  );
  let rot = 0;

  p.setup = () => {
    p.createCanvas(500, 500);
  };

  p.draw = () => {
    if (p.keyIsPressed) {
      switch (p.key) {
      case 'a':
        rot -= Math.PI / 180;
        break;
      case 'd':
        rot += Math.PI / 180;
        break;
      }
    }
    const poly2 = Polygon.regular(
      5, new Vec2(p.mouseX, p.mouseY), 80, rot
    );
    const col = Collision.polygon_polygon(poly1, poly2);

    p.background(0);
    p.strokeWeight(0);
    p.fill(192);
    drawPolygon(poly1);
    p.fill((col === undefined) ? 192 : 255);
    drawPolygon(poly2);

    if (col !== undefined) {
      const normal = col.normal.times(col.depth);
      p.strokeWeight(5);

      p.stroke(255, 0, 0);
      for (const v of col.points2) {
        const vn = v.add(normal);
        p.line(v.x, v.y, vn.x, vn.y);
      }
    }
  };
};

let myp5 = new p5(sketch);