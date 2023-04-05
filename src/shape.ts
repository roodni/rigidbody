import {Vec2, Mat2} from './utils'



export abstract class Shape {
  abstract projection(axis: Vec2): [number, number];
  abstract euclidean(angle: number, trans: Vec2): Shape;

  // 重心・面積・慣性モーメントを計算する
  abstract areas(): {center: Vec2, area: number, inertia: number};

  abstract collide(s2: Shape): Collision | undefined;
  abstract collidePolygon(p1: Polygon): Collision | undefined;
  abstract collideCircle(c1: Circle): Collision | undefined;
}

export class Polygon extends Shape {
  vertices: Vec2[]; // 時計回り
  axes: Vec2[]; // 法線
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
    if (index === -1) {
      index = this.vertices.length - 1;
    } else if (index === this.vertices.length) {
      index = 0;
    }
    return this.vertices[index];
  }

  projection(axis: Vec2): [number, number] {
    let min = Infinity;
    let max = -Infinity;
    for (let i = 0; i < this.vertices.length; i++) {
      const p = axis.dot(this.vertices[i]);
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

  areas() {
    const v0 = this.vertex(0);

    let subAs = []; // 分割した三角形の面積
    let subGs = []; // 分割した重心位置
    let subIs = []; // 分割した慣性モーメント
    let area = 0;
    let v0G = Vec2.zero();

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

  /**
   * 辺で分離軸判定
   * 最も貫通の浅い辺について貫通深度・辺番号を返す
  */
  shallowestEdge(shape: Shape) {
    let minDepth = Infinity;
    let minAxisI = 0;
    for (let i = 0; i < this.axes.length; i++) {
      const axis = this.axes[i];
      const [l1, r1] = this.projection(axis);
      const [l2, r2] = shape.projection(axis);
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

  collide(s2: Shape) { return s2.collidePolygon(this); }
  collidePolygon(p1: Polygon) {
    return Collision.polygonPolygon(p1, this);
  }
  collideCircle(c1: Circle): Collision | undefined {
    return Collision.polygonCircle(this, c1)?.reverse();
  }
}

export class Circle extends Shape {
  constructor(
    public center: Vec2,
    public radius: number
  ) {
    super();
  }

  projection(axis: Vec2): [number, number] {
    const p = axis.dot(this.center);
    return [p - this.radius, p + this.radius];
  }
  euclidean(angle: number, trans: Vec2) {
    return new Circle(this.center.add(trans), this.radius);
  }

  areas() {
    const area = Math.PI * this.radius**2;
    const inertia = area * this.radius**2 / 2;
    return {
      area, inertia,
      center: this.center,
    };
  }

  collide(s2: Shape) { return s2.collideCircle(this); }
  collidePolygon(p1: Polygon): Collision | undefined {
    return Collision.polygonCircle(p1, this);
  }
  collideCircle(c1: Circle): Collision | undefined {
    return Collision.circleCircle(c1, this);
  }

}


export class Collision {
  constructor(
    public normal: Vec2,  // 図形1から図形2への方向の衝突法線
    public points: [Vec2, Vec2, number][],
      // 衝突点と貫通深度
      // 法線から見て左側から並ぶことが保証される
  ) {}

  /** 図形1と図形2をin-placeで入れ替える */
  reverse() {
    this.normal = this.normal.reverse();
    for (let i = 0; i < this.points.length; i++) {
      const [p1, p2, _] = this.points[i];
      this.points[i][0] = p2;
      this.points[i][1] = p1;
    }
    this.points.reverse();
    return this;
  }

  static polygonCircle(poly1: Polygon, circ2: Circle) {
    // ポリゴンの各辺について分離軸判定
    const se = poly1.shallowestEdge(circ2);
    if (!se) { return undefined; }

    // 円に最も近い頂点を求める
    let minDistSq = Infinity;
    let nearestI = 0
    for (let i = 0; i < poly1.vertices.length; i++) {
      const v = poly1.vertices[i];
      const distSq = v.to(circ2.center).normSq();
      if (minDistSq > distSq) {
        minDistSq = distSq;
        nearestI = i;
      }
    }
    const v1 = poly1.vertices[nearestI];
    const v12 = v1.to(circ2.center);
    const d12 = Math.sqrt(minDistSq);
    // 法線を求める
    let normal1 = undefined;
    if (d12 > Number.EPSILON) {
      normal1 = v12.times(1 /  d12);
    }
    if (normal1) {
      // 円に最も近い頂点が射影の端でなければ、その法線は無視
      const vLeft = poly1.vertex(nearestI - 1);
      const vRight = poly1.vertex(nearestI + 1)
      const p1 = normal1.dot(v1);
      const pL = normal1.dot(vLeft);
      const pR = normal1.dot(vRight);
      if (p1 <= pL || p1 <= pR) {
        normal1 = undefined;
      }
    }
    // 法線が有効であれば分離軸判定
    if (normal1) {
      const depth1 = circ2.radius - d12;
      if (depth1 < 0) {
        return undefined; // 分離軸だった
      }
      if (depth1 < se.depth) {
        // 最も貫通の浅い軸だった
        return new Collision(normal1, [
          [v1, circ2.center.add(normal1.times(-circ2.radius)), depth1]
        ]);
      }
    }
    // ポリゴンの辺が最も浅い軸だった
    const normal = poly1.axes[se.axisI];
    return new Collision(normal, [
      [
        circ2.center.add(normal.times(se.depth -circ2.radius)),
        circ2.center.add(normal.times(-circ2.radius)),
        se.depth
      ]
    ]);
  }

  static circleCircle(circ1: Circle, circ2: Circle) {
    const p12 = circ1.center.to(circ2.center);
    const d12 = p12.norm();
    const depth = circ1.radius + circ2.radius - d12;
    if (depth < 0) {
      return undefined;
    }
    let normal;
    if (d12 < Number.EPSILON) {
      normal = Vec2.c(1, 0);
    } else {
      normal = p12.times(1 / d12);
    }
    return new Collision(normal, [
      [
        circ1.center.add(normal.times(circ1.radius)),
        circ2.center.add(normal.times(-circ2.radius)),
        depth
      ]
    ]);
  }

  static polygonPolygon(poly1: Polygon, poly2: Polygon) {
    const se1 = poly1.shallowestEdge(poly2);
    if (!se1) return undefined;
    const se2 = poly2.shallowestEdge(poly1);
    if (!se2) return undefined;

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
    const points: [Vec2, Vec2, number][] = [];
    if (edgePerpPos < vs2[1].p) {
      // 1点衝突
      points.push([
        vs2[0].v.add(normal.times(depth)),
        vs2[0].v,
        depth
      ]);
    } else {
      // 2点衝突
      const edge = normal.rot90();
      const perpPos2a = vs2[0].p;
      const perpPos2b = vs2[1].p;
      const paraPos2a = edge.dot(vs2[0].v);
      const paraPos2b = edge.dot(vs2[1].v);
      // 衝突候補点を接触面に沿った位置で並べて、内側の2つを取り出す
      const paraPoss = [
        edge.dot(poly1.vertex(axisI)),
        edge.dot(poly1.vertex(axisI + 1)),
        paraPos2a,
        paraPos2b,
      ];
      paraPoss.sort((a, b) => a - b);
      for (const paraPos of [paraPoss[1], paraPoss[2]]) {
        // ポリゴン2側の衝突点の算出
        let perpPos2 = edgePerpPos - depth;
        if (Math.abs(paraPos2b - paraPos2a) > Number.EPSILON) {
          perpPos2 = perpPos2a +(perpPos2b -perpPos2a)/(paraPos2b -paraPos2a)*(paraPos -paraPos2a);
        }
        const para = edge.times(paraPos);
        points.push([
          para.add(normal.times(edgePerpPos)),
          para.add(normal.times(perpPos2)),
          edgePerpPos - perpPos2
        ]);
      }
    }

    // おわり
    const col = new Collision(normal, points);
    if (reversed) {
      col.reverse();
    }
    return col;
  }
}