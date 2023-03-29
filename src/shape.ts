import {Vec2, Mat2} from './utils'

export abstract class Shape {
  abstract projection(axis: Vec2): [number, number];
  abstract euclidean(angle: number, trans: Vec2): Shape;

  // 重心・面積・慣性モーメントを計算する
  abstract center(): {center: Vec2, area: number, inertia: number};

  // abstract collide(s: Shape): Collision;
  // abstract collidePolygon(poly: Polygon): Collision;
}

export class Polygon extends Shape {
  vertices: Vec2[]; // 時計回り
  axes: Vec2[];
  constructor(vs: Vec2[]) {
    super();
    this.vertices = vs;
    this.axes = [];
    for (let i = 0; i < vs.length; i++) {
      const v1 = vs[i];
      const v2 = vs[(i + 1) % vs.length];
      const n = v1.to(v2).rot270().normalize();
      this.axes.push(n);
    }
  }

  /** 頂点を取得する (ラップアラウンドする) */
  vertex(index: number) {
    return this.vertices[index % this.vertices.length];
  }

  projection(axis: Vec2): [number, number] {
    let min = Infinity;
    let max = -Infinity;
    for (const v of this.vertices) {
      const p = axis.dot(v);
      min = Math.min(min, p);
      max = Math.max(max, p);
    }
    return [min, max];
  }

  euclidean(angle: number, trans: Vec2) {
    const rot = Mat2.rotate(angle);
    const l = this.vertices.map((v) => {
      return rot.mulvec(v).add(trans);
    });
    return new Polygon(l);
  }

  center() {
    const v0 = this.vertex(0);

    let subAs = []; // 分割した三角形の面積
    let subGs = []; // 分割した重心位置
    let subIs = []; // 分割した慣性モーメント
    let area = 0;
    let v0G = Vec2.ZERO;

    for (let i = 0; i < this.vertices.length; i++) {
      const va = this.vertex(i).sub(v0);
      const vb = this.vertex(i + 1).sub(v0);
      const subA = Math.abs(va.cross(vb)) / 2;
      const subG = va.add(vb).times(1/3);
      subAs.push(subA);
      subGs.push(subG);
      subIs.push(subA/18*(va.normSq() +vb.normSq() -va.dot(vb)));
      area += subA;
      v0G = v0G.add(subG.times(subA));
    }
    v0G = v0G.times(1/area);

    let inertia = 0;
    for (let i = 0; i < this.vertices.length; i++) {
      const d2 = subGs[i].to(v0G).normSq();
      inertia += subIs[i] +subAs[i]*d2;
    }

    return {
      center: v0.add(v0G),
      area, inertia
    };
  }

  /** 正多角形 */
  static regular(n: number, center: Vec2, radius: number, rot=0) {
    const l: Vec2[] = [];
    const base = Vec2.c(radius, 0);
    for (let i = 0; i < n; i++) {
       const v = Mat2.rotate(rot + Math.PI*2*i/n).mulvec(base).add(center);
       l.push(v);
    }
    return new Polygon(l);
  }

  /** `v1`から`v2`への壁を作る (時計回り) */
  static wall(v1: Vec2, v2: Vec2, width: number) {
    const perp = v1.to(v2).normalize().rot270().times(-width);
    const v3 = v2.add(perp);
    const v4 = v1.add(perp);
    return new Polygon([v1, v2, v3, v4]);
  }

  static rect(x1: number, y1: number, x2: number, y2: number) {
    [x1, x2] = [Math.min(x1, x2), Math.max(x1, x2)];
    [y1, y2] = [Math.min(y1, y2), Math.max(y1, y2)];
    return new Polygon([
      Vec2.c(x1, y1),
      Vec2.c(x2, y1),
      Vec2.c(x2, y2),
      Vec2.c(x1, y2)
    ]);
  }
}

export class Collision {
  normal: Vec2; // 図形1から図形2への方向の衝突法線
  depth: number;
  points: [Vec2, Vec2][];
  constructor(normal: Vec2, depth: number, points: [Vec2, Vec2][]) {
    this.normal = normal;
    this.depth = depth;
    this.points = points;
  }

  reverse() {
    this.normal = this.normal.reverse();
    for (let i = 0; i < this.points.length; i++) {
      const [p1, p2] = this.points[i];
      this.points[i] = [p2, p1];
    }
  }

  static polygon_polygon(poly1: Polygon, poly2: Polygon) {
    function shallowestEdge(polyAxis: Polygon, polyOther: Polygon) {
      // 分離軸判定の片側
      // 最も貫通の浅い軸について貫通深度・辺番号
      // 分離軸が存在したら打ち切る
      let minDepth = Infinity;
      let minAxisI = 0;
      for (let i = 0; i < polyAxis.axes.length; i++) {
        const axis = polyAxis.axes[i];
        const [l1, r1] = polyAxis.projection(axis);
        const [l2, r2] = polyOther.projection(axis);
        const depth = r1 - l2;
        if (depth < 0 || (r1 -l1) +(r2 -l2) < depth) {
          return undefined;
        }
        if (depth < minDepth) {
          minDepth = depth;
          minAxisI = i;
        }
      }
      return {
        depth: minDepth,
        axisI: minAxisI
      };
    }

    const se1 = shallowestEdge(poly1, poly2);
    if (se1 === undefined) return undefined;
    const se2 = shallowestEdge(poly2, poly1);
    if (se2 === undefined) return undefined;

    let reversed = false;
    let se;
    if (se1.depth <= se2.depth) {
      se = se1;
    } else {
      reversed = true;
      se = se2;
      [poly1, poly2] = [poly2, poly1];
    }
    const {depth, axisI} = se;
    const normal = poly1.axes[axisI];
    const edgePerpPos = poly1.vertex(axisI).dot(normal);  // 法線に沿った辺の位置
    // 以下、poly1は衝突軸の元になったポリゴン、poly2はもう一方のポリゴン

    // もう一方のポリゴンから頂点をめり込みが大きい順に1つまたは2つ持ってくる
    const vs2 = poly2.vertices.map((v) => {
      const p = v.dot(normal);
      return {p, v};
    });
    vs2.sort((a, b) => a.p - b.p);

    // 衝突点を算出する
    const points: [Vec2, Vec2][] = [];
    if (edgePerpPos < vs2[1].p) {
      // 1点衝突
      points.push([
        vs2[0].v.add(normal.times(depth)),
        vs2[0].v
      ]);
    } else {
      // 2点衝突
      const edge = normal.rot270();
      // 衝突候補点を接触面に沿った位置で並べて、内側の2つを取り出す
      const paraPoss = [
        poly1.vertex(axisI),
        poly1.vertex(axisI + 1),
        vs2[0].v,
        vs2[1].v,
      ].map((v) => edge.dot(v));
      paraPoss.sort((a, b) => a - b);
      for (const paraPos of [paraPoss[1], paraPoss[2]]) {
        const v = edge.times(paraPos);
        points.push([
          normal.times(edgePerpPos).add(v),
          normal.times(edgePerpPos -depth).add(v)
        ]);
      }
    }

    // おわり
    const col = new Collision(normal, depth, points);
    if (reversed) {
      col.reverse();
    }
    return col;
  }
}