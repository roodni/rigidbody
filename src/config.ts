import * as scenes from './scene';


const sceneList: ((w: number, h: number) => scenes.Scene)[] = [
  (w, h) => new scenes.LoopScene1(w, h),
  (w, h) => new scenes.LoopScene2(w, h),
  (w, h) => new scenes.BodiesScene(w, h),
  (w, h) => new scenes.StackScene(w, h),
  (w, h) => new scenes.BoxedWorldScene(w, h),
  (w, h) => new scenes.PendulumScene(w, h),
  () => new scenes.CollisionScene(),
];

const ShapeName = ['rectangle', 'circle', 'delete'] as const;
type ShapeName = typeof ShapeName[number];

const config = {
  CANVAS_W: 500,
  CANVAS_H: 500,

  _scene: undefined as (scenes.Scene | undefined),
  setScene(idx: number) {
    this._scene = sceneList[idx](this.CANVAS_W, this.CANVAS_H);
    this._scene.init();
    return this._scene;
  },
  get scene() {
    let scene = this._scene;
    if (!scene) {
      scene = this.setScene(0);
    }
    return scene;
  },

  shapeName: ShapeName[0] as ShapeName,
  freeze: false,

  restitution: 0.5,
  friction: 0.5
};
export default config;


window.addEventListener('load', () => {
  // シーン切り替え
  const sceneSel = document.querySelector<HTMLSelectElement>('#scene')!;
  for (let i = 0; i < sceneList.length; i++) {
    const opt = document.createElement('option');
    opt.value = i.toString();
    opt.text = `scene ${i}`;
    sceneSel.add(opt);
  }
  sceneSel.addEventListener('change', () => {
    config.setScene(parseInt(sceneSel.value));
  });
  document.querySelector('#reset')!.addEventListener('click', () => {
    config.setScene(parseInt(sceneSel.value));
  });

  // 生成する図形の種類
  const shapeNameCntnr = document.querySelector('#shape_name_cntnr')!;
  for (const name of ShapeName) {
    const ipt = document.createElement('input');
    const lbl = document.createElement('label');
    ipt.type = 'radio';
    ipt.name = 'shape_name';
    if (name === config.shapeName) {
      ipt.checked = true;
    }
    ipt.addEventListener('change', () => {
      config.shapeName = name;
    });
    lbl.append(ipt, name);
    shapeNameCntnr.appendChild(lbl);
  }

  // フリーズ
  const freezeSel = document.querySelector<HTMLInputElement>('#freeze')!;
  freezeSel.checked = config.freeze;
  freezeSel.addEventListener('change', () => {
    config.freeze = freezeSel.checked;
  });

  // 反発係数と摩擦係数
  const restitutionSel = document.querySelector<HTMLInputElement>('#restitution')!;
  restitutionSel.valueAsNumber = config.restitution;
  restitutionSel.addEventListener('change', () => {
    config.restitution = restitutionSel.valueAsNumber;
  })

  const frictionSel = document.querySelector<HTMLInputElement>('#friction')!;
  frictionSel.valueAsNumber = config.friction;
  frictionSel.addEventListener('change', () => {
    config.friction = frictionSel.valueAsNumber;
  });
});