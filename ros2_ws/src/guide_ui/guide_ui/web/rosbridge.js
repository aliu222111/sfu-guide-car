// Minimal rosbridge v2 client: publish and subscribe over one WebSocket, reconnecting.
class Ros {
  constructor(url) {
    this.url = url; this.subs = new Map(); this.adverts = new Map(); this.onstatus = () => {};
    this.connect();
  }
  connect() {
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => {
      this.onstatus(true);
      for (const [topic, type] of this.adverts) this.send({ op: "advertise", topic, type });
      for (const [topic, s] of this.subs) this.send({ op: "subscribe", topic, type: s.type });
    };
    this.ws.onclose = () => { this.onstatus(false); setTimeout(() => this.connect(), 1500); };
    this.ws.onerror = () => this.ws.close();
    this.ws.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      if (m.op === "publish" && this.subs.has(m.topic)) this.subs.get(m.topic).cb(m.msg);
    };
  }
  send(obj) { if (this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj)); }
  advertise(topic, type) { this.adverts.set(topic, type); this.send({ op: "advertise", topic, type }); }
  publish(topic, type, msg) {
    if (!this.adverts.has(topic)) this.advertise(topic, type);
    this.send({ op: "publish", topic, msg });
  }
  subscribe(topic, type, cb) { this.subs.set(topic, { type, cb }); this.send({ op: "subscribe", topic, type }); }
}
