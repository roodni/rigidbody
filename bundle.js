(()=>{"use strict";var t,e,s,i={},o={};function r(t){var e=o[t];if(void 0!==e)return e.exports;var s=o[t]={exports:{}};return i[t](s,s.exports,r),s.exports}r.m=i,e=Object.getPrototypeOf?t=>Object.getPrototypeOf(t):t=>t.__proto__,r.t=function(s,i){if(1&i&&(s=this(s)),8&i)return s;if("object"==typeof s&&s){if(4&i&&s.__esModule)return s;if(16&i&&"function"==typeof s.then)return s}var o=Object.create(null);r.r(o);var n={};t=t||[null,e({}),e([]),e(e)];for(var a=2&i&&s;"object"==typeof a&&!~t.indexOf(a);a=e(a))Object.getOwnPropertyNames(a).forEach((t=>n[t]=()=>s[t]));return n.default=()=>s,r.d(o,n),o},r.d=(t,e)=>{for(var s in e)r.o(e,s)&&!r.o(t,s)&&Object.defineProperty(t,s,{enumerable:!0,get:e[s]})},r.f={},r.e=t=>Promise.all(Object.keys(r.f).reduce(((e,s)=>(r.f[s](t,e),e)),[])),r.u=t=>t+".bundle.js",r.g=function(){if("object"==typeof globalThis)return globalThis;try{return this||new Function("return this")()}catch(t){if("object"==typeof window)return window}}(),r.o=(t,e)=>Object.prototype.hasOwnProperty.call(t,e),s={},r.l=(t,e,i,o)=>{if(s[t])s[t].push(e);else{var n,a;if(void 0!==i)for(var h=document.getElementsByTagName("script"),c=0;c<h.length;c++){var d=h[c];if(d.getAttribute("src")==t){n=d;break}}n||(a=!0,(n=document.createElement("script")).charset="utf-8",n.timeout=120,r.nc&&n.setAttribute("nonce",r.nc),n.src=t),s[t]=[e];var l=(e,i)=>{n.onerror=n.onload=null,clearTimeout(u);var o=s[t];if(delete s[t],n.parentNode&&n.parentNode.removeChild(n),o&&o.forEach((t=>t(i))),e)return e(i)},u=setTimeout(l.bind(null,void 0,{type:"timeout",target:n}),12e4);n.onerror=l.bind(null,n.onerror),n.onload=l.bind(null,n.onload),a&&document.head.appendChild(n)}},r.r=t=>{"undefined"!=typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(t,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(t,"__esModule",{value:!0})},(()=>{var t;r.g.importScripts&&(t=r.g.location+"");var e=r.g.document;if(!t&&e&&(e.currentScript&&(t=e.currentScript.src),!t)){var s=e.getElementsByTagName("script");s.length&&(t=s[s.length-1].src)}if(!t)throw new Error("Automatic publicPath is not supported in this browser");t=t.replace(/#.*$/,"").replace(/\?.*$/,"").replace(/\/[^\/]+$/,"/"),r.p=t})(),(()=>{var t={179:0};r.f.j=(e,s)=>{var i=r.o(t,e)?t[e]:void 0;if(0!==i)if(i)s.push(i[2]);else{var o=new Promise(((s,o)=>i=t[e]=[s,o]));s.push(i[2]=o);var n=r.p+r.u(e),a=new Error;r.l(n,(s=>{if(r.o(t,e)&&(0!==(i=t[e])&&(t[e]=void 0),i)){var o=s&&("load"===s.type?"missing":s.type),n=s&&s.target&&s.target.src;a.message="Loading chunk "+e+" failed.\n("+o+": "+n+")",a.name="ChunkLoadError",a.type=o,a.request=n,i[1](a)}}),"chunk-"+e,e)}};var e=(e,s)=>{var i,o,[n,a,h]=s,c=0;if(n.some((e=>0!==t[e]))){for(i in a)r.o(a,i)&&(r.m[i]=a[i]);h&&h(r)}for(e&&e(s);c<n.length;c++)o=n[c],r.o(t,o)&&t[o]&&t[o][0](),t[o]=0},s=self.webpackChunk=self.webpackChunk||[];s.forEach(e.bind(null,0)),s.push=e.bind(null,s.push.bind(s))})();class n{constructor(t,e){this.x=t,this.y=e}static c(t,e){if(!isFinite(t)||!isFinite(e))throw new Error("not finite");return new n(t,e)}static zero(){return new n(0,0)}copy(){return new n(this.x,this.y)}copyMut(t){return this.x=t.x,this.y=t.y,this}update(t,e){this.x=t,this.y=e}zero(){return this.x=0,this.y=0,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}normSq(){return this.x**2+this.y**2}norm(){return Math.sqrt(this.normSq())}normalize(){const t=this.norm();return n.c(this.x/t,this.y/t)}safeNormalize(t){const e=this.norm();return e<Number.EPSILON?t:n.c(this.x/e,this.y/e)}rot90(){return new n(-this.y,this.x)}rotMut90(){const t=this.x;return this.x=-this.y,this.y=t,this}rot270(){return new n(this.y,-this.x)}rotMut270(){const t=this.x;return this.x=this.y,this.y=-t,this}reverse(){return n.c(-this.x,-this.y)}reverseMut(){return this.x=-this.x,this.y=-this.y,this}times(t){return n.c(this.x*t,this.y*t)}timesMut(t){return this.x*=t,this.y*=t,this}add(t){return n.c(this.x+t.x,this.y+t.y)}addMut(t){return this.x+=t.x,this.y+=t.y,this}linear(t,e){return n.c(this.x+t.x*e,this.y+t.y*e)}linearMut(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}addRot90Mut(t,e){return this.x+=-t.y*e,this.y+=t.x*e,this}sub(t){return n.c(this.x-t.x,this.y-t.y)}subMut(t){return this.x-=t.x,this.y-=t.y,this}to(t){return t.sub(this)}toTuple(){return[this.x,this.y]}}class a{constructor(t,e,s,i){this.a=t,this.b=e,this.c=s,this.d=i}mulvec(t){return n.c(this.a*t.x+this.b*t.y,this.c*t.x+this.d*t.y)}static rotate(t){const e=Math.cos(t),s=Math.sin(t);return new a(e,-s,s,e)}}function h(t,e){const s=e-t+1;return t+Math.floor(Math.random()*s)}function c(t,e){return t+Math.random()*(e-t)}class d{constructor(t,e,s){this.frozen=!1,this.pos=n.zero(),this.vel=n.zero(),this.acc=n.zero(),this.angle=0,this.avel=0,this.aacc=0,this.restitution=.5,this.friction=.3,this.id=d.idPool++,this.shape=t,this.mass=e,this.invMass=1/e,this.inertia=s,this.invInertia=1/s,this.noCollide=new Set}freeze(){this.frozen=!0,this.invMass=0,this.invInertia=0,this.vel=n.zero(),this.acc=n.zero()}unfreeze(){this.frozen=!1,this.invMass=1/this.mass,this.invInertia=1/this.invInertia}addForceLocal(t,e){this.acc.linearMut(e,this.invMass),this.aacc+=t.cross(e)*this.invInertia}addForce(t,e){const s=this.pos.to(t);this.addForceLocal(s,e)}addImpulseLocal(t,e){this.vel.linearMut(e,this.invMass),this.avel+=t.cross(e)*this.invInertia}addImpulseReverse(t,e){this.vel.linearMut(e,-this.invMass),this.avel-=t.cross(e)*this.invInertia}velAtLocal(t,e){return e?e.zero():e=n.zero(),e.addMut(this.vel).addRot90Mut(t,this.avel),e}velAt(t){const e=this.pos.to(t);return this.velAtLocal(e)}static velRelative(t,e,s,i,o){return o.zero().addMut(e.vel).addRot90Mut(i,e.avel).subMut(t.vel).addRot90Mut(s,-t.avel),o}movedShape(){if(this.shape)return this.shape.euclidean(this.angle,this.pos)}static fromShape(t,e=1){const{area:s,inertia:i,center:o}=t.areas(),r=s*e,n=i*e;t=t.euclidean(0,o.reverse());const a=new d(t,r,n);return a.pos=o,a}static ether(){const t=new d(void 0,1,1);return t.freeze(),t}}d.idPool=0;class l{constructor(t,e,s){this.normal=n.zero(),this.tangent=n.zero(),this.r1=n.zero(),this.r2=n.zero(),this.impulse=n.zero(),this.goalVel=0,this.invMassN=0,this.invMassT=0,this.invMassNT=0,this.temp=n.zero(),this.isFrictionStatic=!1,this.body1=t,this.body2=e,this.pointi=s}update(t,e){const[s,i,o]=e.points[this.pointi],r=this.normal.copyMut(e.normal),n=this.tangent.copyMut(e.normal).rotMut90(),a=this.body1,h=this.body2;this.r1.copyMut(s).addMut(i).timesMut(.5);const c=this.r2.copyMut(this.r1).subMut(h.pos),l=this.r1.subMut(a.pos),u=d.velRelative(a,h,l,c,this.temp).dot(r);let p=a.restitution*h.restitution;const m=this.temp.zero().addMut(h.acc).addRot90Mut(c,h.aacc).subMut(a.acc).addRot90Mut(l,-a.aacc),f=Math.min(0,r.dot(m)*t),v=Math.max(0,-p*u+f),y=9.8*t*t*3,w=Math.min(o-y,y)/t;this.goalVel=Math.max(v,w,0),this.invMassN=a.invMass+h.invMass+a.invInertia*l.cross(r)**2+h.invInertia*c.cross(r)**2,this.invMassT=a.invMass+h.invMass+a.invInertia*l.cross(n)**2+h.invInertia*c.cross(n)**2,this.invMassNT=a.invInertia*l.cross(r)*l.cross(n)+h.invInertia*c.cross(r)*c.cross(n)}warmStart(){this.body1.addImpulseReverse(this.r1,this.impulse),this.body2.addImpulseLocal(this.r2,this.impulse)}solve(){const t=this.r1,e=this.r2,s=this.normal,i=this.tangent,o=this.invMassN,r=this.invMassT,n=this.invMassNT;this.body1.addImpulseLocal(t,this.impulse),this.body2.addImpulseReverse(e,this.impulse),this.impulse.zero();const a=d.velRelative(this.body1,this.body2,t,e,this.temp),h=this.goalVel-s.dot(a);if(h<=0)return;let c=-(o*i.dot(a)+n*h)/(r*o-n**2),l=(h-n*c)/o;const u=this.body1.friction*this.body2.friction;if(this.isFrictionStatic=!0,Math.abs(c)>u*l){this.isFrictionStatic=!1;const t=Math.sign(c);l=h/(o+t*u*n),c=t*u*l}this.impulse.linearMut(i,c).linearMut(s,l),this.body1.addImpulseReverse(this.r1,this.impulse),this.body2.addImpulseLocal(this.r2,this.impulse)}}class u{constructor(){this.map=new Map}clear(){this.map.clear()}find(t,e){var s;return null===(s=this.map.get(t))||void 0===s?void 0:s.get(e)}add(t){let e=this.map.get(t.body1.id);e||(e=new Map,this.map.set(t.body1.id,e));let s=e.get(t.body2.id);s||(s=[],e.set(t.body2.id,s)),s[t.pointi]=t}}class p{constructor(t,e,s,i,o){var r;const a=this.body1=t,h=this.body2=e;this.goalVel=i;const c=this.r1=a.pos.to(s),d=this.r2=h.pos.to(s);this.invMassX=a.invMass+h.invMass+a.invInertia*c.y**2+h.invInertia*d.y**2,this.invMassY=a.invMass+h.invMass+a.invInertia*c.x**2+h.invInertia*d.x**2,this.invMassXY=-a.invInertia*c.x*c.y-h.invInertia*d.x*d.y,this.impulse=null!==(r=null==o?void 0:o.impulse)&&void 0!==r?r:n.zero(),this.velDiff=n.zero()}warmStart(){this.body1.addImpulseReverse(this.r1,this.impulse),this.body2.addImpulseLocal(this.r2,this.impulse)}solve(){d.velRelative(this.body1,this.body2,this.r1,this.r2,this.velDiff);const t=this.velDiff.reverseMut().addMut(this.goalVel),e=this.impulse.x,s=this.impulse.y,i=this.impulse;i.x=this.invMassY*t.x-this.invMassXY*t.y,i.y=-this.invMassXY*t.x+this.invMassX*t.y,i.timesMut(1/(this.invMassX*this.invMassY-this.invMassXY**2)),this.body1.addImpulseReverse(this.r1,i),this.body2.addImpulseLocal(this.r2,i),i.x+=e,i.y+=s}}class m{constructor(t,e,s,i,o){this.body1=t,this.body2=e,this.r1=s,this.r2=i,this.noCollide=o}pos1(){return a.rotate(this.body1.angle).mulvec(this.r1).add(this.body1.pos)}pos2(){return a.rotate(this.body2.angle).mulvec(this.r2).add(this.body2.pos)}updateConstraint(t){const e=this.pos1(),s=this.pos2(),i=e.to(s),o=i.norm(),r=e.add(i.times(.5)),n=i.times(-o/t);return this.constraint=new p(this.body1,this.body2,r,n,this.constraint),this.constraint}static pin(t,e,s,i=!0){const o=a.rotate(-t.angle),r=a.rotate(-e.angle),n=o.mulvec(t.pos.to(s)),h=r.mulvec(e.pos.to(s));return new m(t,e,n,h,i)}}class f{constructor(t,e){this.body=t,this.shape=e;const[s,i]=e.projection(n.c(1,0)),[o,r]=e.projection(n.c(0,1));this.mins=[s,o],this.maxs=[i,r]}overlap(t,e){return this.mins[t]<=e.maxs[t]&&this.mins[t]<=e.maxs[t]}}class v{constructor(){this.gravity=n.c(0,9.8),this.bodies=[],this.shapes=[],this.joints=[],this.contacts=[],this.contactDict=new u,this.deletionSet=new Set,this.ether=d.ether(),this.addBody(this.ether)}addBody(t){this.bodies.push(t)}addJoint(t){this.joints.push(t),t.noCollide&&(t.body1.noCollide.add(t.body2.id),t.body2.noCollide.add(t.body1.id))}deleteBody(t){this.deletionSet.add(t.id)}step(t){let e=0;for(let t=0;t<this.bodies.length;t++){const s=this.bodies[t];if(this.deletionSet.has(s.id))for(let t=0;t<this.joints.length;t++){const e=this.joints[t];e.body1!==s&&e.body2!==s||(this.joints[t--]=this.joints[this.joints.length-1],this.joints.length--)}else this.bodies[e++]=s}if(this.bodies.length=e,this.deletionSet.clear(),t<=0)return;for(const t of this.bodies)t.addForce(t.pos,this.gravity.times(t.mass));for(const e of this.bodies)e.pos=e.pos.add(e.vel.times(t)),e.vel=e.vel.add(e.acc.times(t)),e.angle+=e.avel*t,e.avel+=e.aacc*t;const s=[];for(const e of this.joints){const i=e.updateConstraint(t);i&&s.push(i)}this.contacts=[],this.shapes=this.bodies.map((t=>t.movedShape()));const i=[];for(let t=0;t<this.bodies.length;t++){const e=this.shapes[t];if(!e)continue;const s=this.bodies[t];i.push(new f(s,e))}i.sort(((t,e)=>t.mins[0]-e.mins[0]));for(let e=0;e<i.length;e++){const o=i[e];for(let r=e+1;r<i.length;r++){const e=i[r];if(o.maxs[0]<e.mins[0])break;let n=o.body,a=e.body;if(n.frozen&&a.frozen)continue;if(n.noCollide.has(a.id))continue;if(!o.overlap(1,e))continue;const h=o.shape.collide(e.shape);if(h){n.id>a.id&&(h.reverse(),[n,a]=[a,n]);const e=this.contactDict.find(n.id,a.id);for(let i=0;i<h.points.length;i++){let o=null==e?void 0:e.at(i);o||(o=new l(n,a,i)),o.update(t,h),s.push(o),this.contacts.push(o)}}}}this.contactDict.clear();for(const t of this.contacts)this.contactDict.add(t);!function(t){for(let e=t.length-1;e>=1;e--){const s=h(0,e);[t[e],t[s]]=[t[s],t[e]]}}(s);for(const t of s)t.warmStart();for(let t=0;t<30;t++)for(const t of s)t.solve();for(const t of this.bodies)t.acc=n.zero(),t.aacc=0}findBody(t,e=(t=>!0)){for(let s=this.shapes.length-1;s>=0;s--){const i=this.bodies[s],o=this.shapes[s];if(o&&e(i)&&t.collide(o))return i}}}class y{}class w extends y{constructor(t){super(),this.vertices=t,this.axes=[];for(let e=0;e<t.length;e++){const s=t[e],i=t[(e+1)%t.length],o=s.to(i).rot270().normalize();this.axes.push(o)}}vertex(t){return-1===t?t=this.vertices.length-1:t===this.vertices.length&&(t=0),this.vertices[t]}projection(t){let e=1/0,s=-1/0;for(let i=0;i<this.vertices.length;i++){const o=t.dot(this.vertices[i]);e=Math.min(e,o),s=Math.max(s,o)}return[e,s]}euclidean(t,e){const s=a.rotate(t),i=this.vertices.map((t=>s.mulvec(t).add(e)));return new w(i)}areas(){const t=this.vertex(0);let e=[],s=[],i=[],o=0,r=n.zero();for(let n=0;n<this.vertices.length;n++){const a=this.vertex(n).sub(t),h=this.vertex(n+1).sub(t),c=Math.abs(a.cross(h))/2,d=a.add(h).times(1/3);e.push(c),s.push(d),i.push(c/18*(a.normSq()+h.normSq()-a.dot(h))),o+=c,r=r.add(d.times(c))}r=r.times(1/o);let a=0;for(let t=0;t<this.vertices.length;t++){const o=s[t].to(r).normSq();a+=i[t]+e[t]*o}return{center:t.add(r),area:o,inertia:a}}static regular(t,e,s,i=0){const o=[],r=n.c(s,0);for(let s=0;s<t;s++){const n=a.rotate(i+2*Math.PI*s/t).mulvec(r).add(e);o.push(n)}return new w(o)}static wall(t,e,s){const i=t.to(e).normalize().rot270().times(-s),o=e.add(i),r=t.add(i);return new w([t,e,o,r])}static rect(t,e,s,i){return[t,s]=[Math.min(t,s),Math.max(t,s)],[e,i]=[Math.min(e,i),Math.max(e,i)],new w([n.c(t,e),n.c(s,e),n.c(s,i),n.c(t,i)])}shallowestEdge(t){let e=1/0,s=0;for(let i=0;i<this.axes.length;i++){const o=this.axes[i],[r,n]=this.projection(o),[a,h]=t.projection(o),c=n-a;if(c<0||n-r+(h-a)<c)return;c<e&&(e=c,s=i)}return{depth:e,axisI:s}}collide(t){return t.collidePolygon(this)}collidePolygon(t){return b.polygonPolygon(t,this)}collideCircle(t){var e;return null===(e=b.polygonCircle(this,t))||void 0===e?void 0:e.reverse()}}class g extends y{constructor(t,e){super(),this.center=t,this.radius=e}projection(t){const e=t.dot(this.center);return[e-this.radius,e+this.radius]}euclidean(t,e){return new g(this.center.add(e),this.radius)}areas(){const t=Math.PI*this.radius**2;return{area:t,inertia:t*this.radius**2/2,center:this.center}}collide(t){return t.collideCircle(this)}collidePolygon(t){return b.polygonCircle(t,this)}collideCircle(t){return b.circleCircle(t,this)}}class b{constructor(t,e){this.normal=t,this.points=e}reverse(){this.normal=this.normal.reverse();for(let t=0;t<this.points.length;t++){const[e,s,i]=this.points[t];this.points[t][0]=s,this.points[t][1]=e}return this.points.reverse(),this}static polygonCircle(t,e){const s=t.shallowestEdge(e);if(!s)return;let i=1/0,o=0;for(let s=0;s<t.vertices.length;s++){const r=t.vertices[s].to(e.center).normSq();i>r&&(i=r,o=s)}const r=t.vertices[o],n=r.to(e.center),a=Math.sqrt(i);let h;if(a>Number.EPSILON&&(h=n.times(1/a)),h){const e=t.vertex(o-1),s=t.vertex(o+1),i=h.dot(r),n=h.dot(e),a=h.dot(s);(i<=n||i<=a)&&(h=void 0)}if(h){const t=e.radius-a;if(t<0)return;if(t<s.depth)return new b(h,[[r,e.center.add(h.times(-e.radius)),t]])}const c=t.axes[s.axisI];return new b(c,[[e.center.add(c.times(s.depth-e.radius)),e.center.add(c.times(-e.radius)),s.depth]])}static circleCircle(t,e){const s=t.center.to(e.center),i=s.norm(),o=t.radius+e.radius-i;if(o<0)return;let r;return r=i<Number.EPSILON?n.c(1,0):s.times(1/i),new b(r,[[t.center.add(r.times(t.radius)),e.center.add(r.times(-e.radius)),o]])}static polygonPolygon(t,e){const s=t.shallowestEdge(e);if(!s)return;const i=e.shallowestEdge(t);if(!i)return;let o,r=!1;s.depth<=i.depth?o=s:(r=!0,o=i,[t,e]=[e,t]);const{depth:n,axisI:a}=o,h=t.axes[a],c=t.vertex(a).dot(h),d=e.vertices.map((t=>({p:t.dot(h),v:t})));d.sort(((t,e)=>t.p-e.p));const l=[];if(c<d[1].p)l.push([d[0].v.linear(h,n),d[0].v,n]);else{const e=h.rot90(),s=d[0].p,i=d[1].p,o=e.dot(d[0].v),r=e.dot(d[1].v),u=[e.dot(t.vertex(a)),e.dot(t.vertex(a+1)),o,r];u.sort(((t,e)=>t-e));for(const t of[u[1],u[2]]){let a=c-n;Math.abs(r-o)>Number.EPSILON&&(a=s+(i-s)/(r-o)*(t-o));const d=e.times(t);l.push([d.linear(h,c),d.linear(h,a),c-a])}}const u=new b(h,l);return r&&u.reverse(),u}}class x{constructor(t,e=n.zero()){this.meterToPx=t,this.origin=e}posToPx(t){return this.origin.to(t).times(this.meterToPx)}pxToPos(t){return t.times(1/this.meterToPx).add(this.origin)}drawPolygon(t,e){t.beginShape();for(const s of e.vertices)t.vertex(...this.posToPx(s).toTuple());t.endShape(t.CLOSE)}drawBody(t,e,s){const i=e.movedShape();if(i){if(t.stroke(s),t.noFill(),i instanceof w)this.drawPolygon(t,i);else if(i instanceof g){const s=this.posToPx(i.center),o=i.radius*this.meterToPx,r=a.rotate(e.angle).mulvec(n.c(o,0)).add(s);t.circle(s.x,s.y,2*o),t.line(s.x,s.y,r.x,r.y)}H.showCenter&&(t.noStroke(),t.fill(s),t.circle(...this.posToPx(e.pos).toTuple(),8))}}drawContact(t,e){const s=this.posToPx(e.body1.pos.add(e.r1)),i=this.posToPx(e.body2.pos.add(e.r2)),o=s.add(i).times(.5);t.colorMode(t.RGB,255);let r=t.color(255,0,0);0==e.impulse.normSq()?r=t.color(0,0,255):e.isFrictionStatic&&(r=t.color(0,255,0)),t.fill(r),t.noStroke(),t.circle(...o.toTuple(),6),t.noFill(),t.stroke(r),t.line(o.x,o.y,...o.add(e.normal.times(6)).toTuple())}drawJoint(t,e){t.colorMode(t.HSB,1);const s=t.color(.5,.2,1);t.noStroke(),t.fill(s);const i=e.pos1(),o=e.pos2();t.circle(...this.posToPx(i.add(o).times(.5)).toTuple(),8)}}class M{mousePressed(t){}mouseReleased(t){}}class S extends M{constructor(){super(),this.body1=d.fromShape(w.regular(4,n.c(250,250),120)),this.angle=0,this.drawer=new x(1)}init(){}update(t){if(t.keyIsPressed)switch(t.key){case"a":this.angle-=Math.PI/180;break;case"d":this.angle+=Math.PI/180}const e=d.fromShape(w.regular(5,n.zero(),80));e.pos=n.c(t.mouseX,t.mouseY),e.angle=this.angle;const s=this.body1.movedShape(),i=e.movedShape(),o=s.collide(i);if(t.colorMode(t.RGB,255),t.background(0),this.drawer.drawBody(t,this.body1,t.color(o?255:192)),this.drawer.drawBody(t,e,t.color(192)),o){t.colorMode(t.RGB,255),t.stroke(255,0,0),t.noFill();for(const[e,s]of o.points)t.line(e.x,e.y,s.x,s.y);const[e,s]=o.points[0];t.circle(e.x,e.y,5)}}}class P extends M{constructor(t,e,s){super(),this.timestep=1/60,this.gripRadius=.03,this.canvasW=t,this.canvasH=e,this.world=new v,this.meterToPx=t/s,this.drawer=new x(this.meterToPx)}updateWorld(){}update(t){var e;this.world.gravity=n.c(0,9.8),t.keyIsPressed&&"g"===t.key&&(this.world.gravity=n.c(0,19.6));const s=this.drawer.pxToPos(n.c(0,5*this.canvasH)).y;for(const t of this.world.bodies)t.pos.y>s&&this.world.deleteBody(t);if(this.updateWorld(),this.world.step(this.timestep),t.colorMode(t.RGB,255),t.background(0),t.mouseIsPressed&&this.grabbing){t.colorMode(t.RGB,255),t.stroke(255),t.noFill();const e=this.grabbing.body,s=this.drawer.pxToPos(n.c(t.mouseX,t.mouseY)),i=e.pos.add(a.rotate(e.angle).mulvec(this.grabbing.loc)),o=e.velAt(i),r=i.to(s),h=r.norm(),c=r.safeNormalize(o.safeNormalize(n.zero())),d=9.8*e.mass*h/.1;e.addForce(i,c.times(d).add(o.times(-9.8*e.mass))),t.circle(...this.drawer.posToPx(i).toTuple(),this.gripRadius*this.meterToPx*2),t.line(t.mouseX,t.mouseY,...this.drawer.posToPx(i).toTuple())}if(t.mouseIsPressed&&!this.grabbing&&this.startPos){t.colorMode(t.RGB,255),t.stroke(255),t.noFill();const e=this.startPos,s=n.c(t.mouseX,t.mouseY);if("rectangle"===H.shapeName)t.rect(e.x,e.y,s.x-e.x,s.y-e.y);else if("circle"===H.shapeName){const i=e.to(s).norm(),o=e.add(s).times(.5);t.circle(o.x,o.y,i)}}if(this.deletionTarget=void 0,t.mouseIsPressed&&"delete"===H.shapeName&&this.startPos){const e=this.drawer.pxToPos(n.c(t.mouseX,t.mouseY)),s=new g(e,this.gripRadius);this.deletionTarget=this.world.findBody(s),t.colorMode(t.RGB,255);const i=t.color(this.deletionTarget?255:128);t.stroke(i),t.noFill(),t.line(0,t.mouseY,this.canvasW,t.mouseY),t.line(t.mouseX,0,t.mouseX,this.canvasH),t.circle(t.mouseX,t.mouseY,this.gripRadius*this.meterToPx*3)}for(const s of this.world.bodies){t.colorMode(t.HSB,1);let i=t.color(0,.2,1);s.id===(null===(e=this.deletionTarget)||void 0===e?void 0:e.id)?i=t.color(0,1,1):s.frozen&&(i=t.color(.6)),this.drawer.drawBody(t,s,i)}for(const e of this.world.joints)this.drawer.drawJoint(t,e);if(H.showCollision)for(const e of this.world.contacts)this.drawer.drawContact(t,e);document.querySelector("#body_count").innerHTML=(this.world.bodies.length-1).toString(),document.querySelector("#joint_count").innerHTML=this.world.joints.length.toString()}mousePressed(t){if(t.mouseX<0||this.canvasW<t.mouseX||t.mouseY<0||this.canvasH<t.mouseY)return void(this.startPos=void 0);if(this.startPos=n.c(t.mouseX,t.mouseY),this.grabbing=void 0,"delete"===H.shapeName)return;const e=this.drawer.pxToPos(this.startPos),s=new g(e,this.gripRadius),i=this.world.findBody(s,(t=>!t.frozen));if(i){const t=a.rotate(-i.angle).mulvec(i.pos.to(e));this.grabbing={body:i,loc:t}}}mouseReleased(t){if(this.deletionTarget)return void this.world.deleteBody(this.deletionTarget);if(this.grabbing)return void(this.grabbing=void 0);if(!this.startPos)return;const e=this.drawer.pxToPos(this.startPos),s=this.drawer.pxToPos(n.c(t.mouseX,t.mouseY));let i;if("rectangle"===H.shapeName){const t=Math.abs(e.x-s.x),o=Math.abs(e.y-s.y);if(Math.min(t,o)<.05)return;i=w.rect(...e.toTuple(),...s.toTuple())}else if("circle"===H.shapeName){const t=e.add(s).times(.5),o=e.to(s).norm()/2;if(o<.05)return;i=new g(t,o)}if(i){const t=d.fromShape(i);H.freeze&&t.freeze(),t.restitution=H.restitution,t.friction=H.friction,this.world.addBody(t)}}}class T extends P{constructor(t,e,s=2,i=2,o=.1){super(t,e,s),this.worldW=s,this.worldH=i,this.margin=o}init(){const t=this.margin,e=t,s=this.worldW-t,i=t,o=this.worldH-t;[w.wall(n.c(e,o),n.c(s,o),t),w.wall(n.c(s,o),n.c(s,i),t),w.wall(n.c(s,i),n.c(e,i),t),w.wall(n.c(e,i),n.c(e,o),t)].forEach((t=>{const e=d.fromShape(t);e.restitution=.9,e.friction=1,e.freeze(),this.world.addBody(e)}))}}function z(t,e,s,[i,o]=[.1,.2]){const r=[];for(let a=0;a<t;a++){const t=h(2,6),a=c(i,o),l=n.c(c(e.x+a,s.x-a),c(e.y+a,s.y-a));let u;if(t<=2)u=new g(l,a);else{const e=h(3,6);u=w.regular(t,l,a,e)}const p=d.fromShape(u);p.restitution=H.restitution,p.friction=H.friction,r.push(p)}return r}class I extends T{constructor(t,e){super(t,e)}init(){super.init();const t=this.margin,e=this.worldW-t,s=this.worldH-t;z(20,n.c(t,t),n.c(e,s),[.1,.2]).forEach((t=>{this.world.addBody(t)}))}updateWorld(){}}class N extends T{constructor(t,e){super(t,e,8,8,.5)}init(){super.init();const t=this.margin,e=this.worldW-t,s=this.worldH-t;z(1e3,n.c(t,t),n.c(e,s),[.06,.12]).forEach((t=>{this.world.addBody(t)}))}}class C extends T{constructor(t,e){super(t,e),this.count=0}init(){super.init()}updateWorld(){if(this.count%60==0&&this.count<480){const t=this.worldW/4,e=this.worldW/4*2,s=this.worldH/10,i=w.rect(t,this.margin,e,this.margin+s),o=d.fromShape(i);o.restitution=H.restitution,o.friction=H.friction,this.world.addBody(o)}this.count++}}class E extends T{constructor(t,e){super(t,e)}init(){this.margin;const t=this.worldW,e=this.worldH;let s=this.world.ether;for(let i=0;i<6;i++){const o=.2,r=.05,a=t/2+o*i,h=e/3,c=w.rect(a,h,a+o,h+r),l=d.fromShape(c);this.world.addBody(l);const u=m.pin(s,l,n.c(a,h+r/2));this.world.addJoint(u),s=l}}}class B extends P{constructor(t,e,s=2,i=2,o=.1){super(t,e,s),this.restitution=.9,this.friction=1,this.worldW=s,this.worldH=i,this.margin=o}init(){const t=this.margin,e=this.worldW,s=this.worldH,i=t,o=e-t;[w.wall(n.c(i,-s),n.c(i,2*s),t),w.wall(n.c(o,2*s),n.c(o,-s),t),w.wall(n.c(o,-s),n.c(i,-s),t)].forEach((t=>{const e=d.fromShape(t);e.restitution=this.restitution,e.friction=this.friction,e.freeze(),this.world.addBody(e)}))}updateWorld(){for(const t of this.world.bodies)t.pos.y>1.5*this.worldH&&(t.pos=n.c(t.pos.x,t.pos.y-1.6*this.worldH))}}class j extends B{init(){super.init();const t=this.worldW,e=this.worldH,s=this.margin,i=e/2,o=t-s,r=e-s;[w.wall(n.c(s,r),n.c(t/10*4,r),s),w.wall(n.c(t/10*7,r),n.c(o,r),s),w.wall(n.c(t/4,i),n.c(t/4*3,i),s)].forEach((t=>{const e=d.fromShape(t);e.restitution=this.restitution,e.friction=this.friction,e.freeze(),this.world.addBody(e)})),z(20,n.c(s,s),n.c(o,r)).forEach((t=>{this.world.addBody(t)}))}}class R extends B{init(){this.friction=.1,super.init();const t=this.worldW,e=this.worldH,s=this.margin,i=t/2,o=e/2-1.5*s,r=t/2*.8,a=.05,h=d.fromShape(w.rect(i-r,o-a,i+r,o+a)),c=d.fromShape(w.rect(i-a,o-r,i+a,o+r));h.restitution=c.restitution=this.restitution,h.friction=c.friction=this.friction,this.world.addBody(h),this.world.addBody(c),[m.pin(this.world.ether,h,n.c(i,o)),m.pin(h,c,n.c(i-r,o)),m.pin(h,c,n.c(i+r,o))].forEach((t=>{this.world.addJoint(t)})),[w.wall(n.c(0,e-2*s),n.c(t/3,e-s),s),w.wall(n.c(t/3*2,e-s),n.c(t,e-2*s),s)].forEach((t=>{const e=d.fromShape(t);e.freeze(),e.restitution=this.restitution,e.friction=this.friction,this.world.addBody(e)})),z(15,n.c(s,0),n.c(t-s,e),[.1,.15]).forEach((t=>{this.world.addBody(t)}))}}const k=[(t,e)=>new j(t,e),(t,e)=>new R(t,e),(t,e)=>new C(t,e),(t,e)=>new I(t,e),(t,e)=>new N(t,e),(t,e)=>new T(t,e),(t,e)=>new E(t,e),()=>new S],L=["rectangle","circle","delete"],_={CANVAS_W:500,CANVAS_H:500,_scene:void 0,setScene(t){return this._scene=k[t](this.CANVAS_W,this.CANVAS_H),this._scene.init(),this._scene},get scene(){let t=this._scene;return t||(t=this.setScene(0)),t},shapeName:L[0],freeze:!1,restitution:.5,friction:.5,showCenter:!1,showCollision:!1},H=_;window.addEventListener("load",(()=>{const t=document.querySelector("#scene");for(let e=0;e<k.length;e++){const s=document.createElement("option");s.value=e.toString(),s.text=`scene ${e}`,t.add(s)}t.addEventListener("change",(()=>{_.setScene(parseInt(t.value))})),document.querySelector("#reset").addEventListener("click",(()=>{_.setScene(parseInt(t.value))}));const e=document.querySelector("#shape_name_cntnr");for(const t of L){const s=document.createElement("input"),i=document.createElement("label");s.type="radio",s.name="shape_name",t===_.shapeName&&(s.checked=!0),s.addEventListener("change",(()=>{_.shapeName=t})),i.append(s,t),e.appendChild(i)}function s(t,e){const s=document.querySelector(t);s.checked=_[e],s.addEventListener("change",(()=>{_[e]=s.checked}))}s("#show_center","showCenter"),s("#show_collision","showCollision"),s("#freeze","freeze");const i=document.querySelector("#restitution");i.valueAsNumber=_.restitution,i.addEventListener("change",(()=>{_.restitution=i.valueAsNumber}));const o=document.querySelector("#friction");o.valueAsNumber=_.friction,o.addEventListener("change",(()=>{_.friction=o.valueAsNumber}))}));const A=[];var W,q;q=function*(){new(0,(yield r.e(35).then(r.t.bind(r,35,23))).default)((t=>{t.setup=()=>{t.createCanvas(H.CANVAS_W,H.CANVAS_H).parent("cvs"),t.strokeWeight(2)},t.draw=()=>{H.scene.update(t);const e=performance.now();if(A.length>60&&A.shift(),A.length>=1){const t=Math.round(A.length/(e-A[0])*1e3);document.querySelector("#fps").innerHTML=t.toString()}A.push(e)},t.mousePressed=()=>{H.scene.mousePressed(t)},t.mouseReleased=()=>{H.scene.mouseReleased(t)}}))},new((W=void 0)||(W=Promise))((function(t,e){function s(t){try{o(q.next(t))}catch(t){e(t)}}function i(t){try{o(q.throw(t))}catch(t){e(t)}}function o(e){var o;e.done?t(e.value):(o=e.value,o instanceof W?o:new W((function(t){t(o)}))).then(s,i)}o((q=q.apply(void 0,[])).next())}))})();