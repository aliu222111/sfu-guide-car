# scout_app: the Sparky Scout rider app (prototype)

The phone interface: map of the Applied Sciences Building (8000, 9000, 10000 levels) and the Student
Union Building 5000 level, search, "Find the nearest", Chat with Scout, and the summon-then-guide trip.
The car, trips and chat answers are simulated inside the page; it is not wired to `guide_api` yet.

- `index.html`: the whole app (template + logic). The backend seam is the `makeApi` object in its script.
- `data/asb-data.js`: rooms, points of interest, stairs and elevators, walkable grids and the map
  artwork, generated from SFU's keyplan drawings.

`index.html` needs one more file beside it, `support.js`: the runtime of the Claude design canvas the
app was built in. It is not ours to redistribute, so it is not in this repository; copy it from the
`sparky-scout-app` folder used for the demo.

```bash
cd scout_app && python -m http.server 8080 --bind 0.0.0.0     # then http://<laptop>:8080 on a phone
```
