// Browser client for the rider API (guide_api). No build step, no dependencies.
//
//   const api = ScoutApi("http://<car laptop>:8090");
//   const car = await api.getCar();
//   const { tripId } = await api.requestTrip({ name: "ASB 9700" }, { name: "ASB 9896" });
//   const stop = api.onTripUpdate(tripId, (u) => console.log(u.phase, u.progress, u.etaSec, u.prompt));
//
// The method names are the ones the Scout app's `this.api` already uses, so
// this object can replace the app's mock for everything the car answers:
// getCar, requestTrip, onTripUpdate, startGuiding, continueTrip, cancelTrip.
// The app keeps its own floor plan, room search and "find the nearest".
//
// A trip update:
//   { tripId, stage: "summon" | "guide",
//     phase: "requested" | "enroute" | "arrived" | "guiding" | "help" | "waiting" | "lost"
//            | "done" | "aborted" | "cancelled" | "failed",
//     progress: 0..1, etaSec, leftM, paused, online,
//     prompt: { kind, title, line, button } | null,   // what the car needs from the rider
//     say: "text the car just spoke", reason,
//     floor: "ASB-09", pose: { x, y, yaw },            // the car, map frame (metres)
//     app: { lv: "9", x, y, heading } | null,          // the car on the app's floor plan
//     route, legs: [ { lv, path: [[x, y], ...] } ] }   // only when the route changes
(function (root) {
  "use strict";

  function ScoutApi(base, options) {
    base = (base || "").replace(/\/+$/, "");
    const token = (options && options.token) || "";

    async function call(method, path, body) {
      const headers = {};
      if (body !== undefined) headers["Content-Type"] = "application/json";
      if (token) headers.Authorization = "Bearer " + token;
      const res = await fetch(base + path, { method, headers, body: body === undefined ? undefined : JSON.stringify(body) });
      const doc = await res.json().catch(() => null);
      if (!res.ok) {
        const err = new Error((doc && doc.message) || "The car did not answer (" + res.status + ").");
        err.code = (doc && doc.error) || "http_" + res.status;
        err.status = res.status;
        throw err;
      }
      return doc;
    }

    const key = (place) => (typeof place === "string" ? place : place && (place.key || place.name)) || "";

    return {
      health: () => call("GET", "/api/health"),

      // { id, name, status: "available" | "busy" | "offline", battery, floor, pose, app, mission }
      getCar: () => call("GET", "/api/car"),

      // Only for a car that cannot localize itself: say which room it is standing at.
      setCarPosition: (room, headingDeg) => call("POST", "/api/car/position", { room: key(room), headingDeg: headingDeg == null ? null : headingDeg }),

      getDestinations: (q) => call("GET", "/api/destinations?q=" + encodeURIComponent(q || "")),

      // from: where the rider is (a room), or null when the rider is already beside the car.
      requestTrip: (from, to) => call("POST", "/api/trips", { pickup: key(from) || null, destination: key(to) }),

      // Calls cb with every update; returns a function that stops listening.
      onTripUpdate(tripId, cb) {
        const url = base + "/api/trips/" + encodeURIComponent(tripId) + "/events" + (token ? "?token=" + encodeURIComponent(token) : "");
        const es = new EventSource(url);
        const ENDED = { done: 1, aborted: 1, cancelled: 1, failed: 1 };
        es.onmessage = (m) => {
          let u;
          try { u = JSON.parse(m.data); } catch (e) { return; }
          cb(u);
          if (ENDED[u.phase]) es.close();
        };
        return () => es.close();
      },

      // Chat with ElevenLabs. Each answer: { heard, reply, audio (base64 MP3 or null), audioType, room: { key, ... } | null }
      chatStatus: () => call("GET", "/api/chat/status"),
      chat: (text, session) => call("POST", "/api/chat", { text: text, session: session || "default" }),
      async chatAudio(blob, session) {
        const headers = { "Content-Type": blob.type || "audio/webm" };
        if (token) headers.Authorization = "Bearer " + token;
        const res = await fetch(base + "/api/chat/audio?session=" + encodeURIComponent(session || "default"), { method: "POST", headers, body: blob });
        const doc = await res.json().catch(() => null);
        if (!res.ok) throw new Error((doc && doc.message) || "The chat did not answer (" + res.status + ").");
        return doc;
      },
      // Plays an answer's audio; resolves when it has finished (or at once if there is none).
      playReply(answer) {
        if (!answer || !answer.audio) return Promise.resolve();
        const a = new Audio("data:" + (answer.audioType || "audio/mpeg") + ";base64," + answer.audio);
        return new Promise((done) => { a.onended = a.onerror = done; a.play().catch(done); });
      },

      startGuiding: (tripId) => call("POST", "/api/trips/" + tripId + "/start"),
      // Answers whatever the car is asking for: door held, elevator called, "still with me".
      confirm: (tripId) => call("POST", "/api/trips/" + tripId + "/confirm"),
      continueTrip: (tripId) => call("POST", "/api/trips/" + tripId + "/confirm"),
      pause: (tripId, paused) => call("POST", "/api/trips/" + tripId + "/pause", { paused: paused !== false }),
      cancelTrip: (tripId) => call("POST", "/api/trips/" + tripId + "/cancel"),
    };
  }

  root.ScoutApi = ScoutApi;
})(typeof window !== "undefined" ? window : globalThis);
