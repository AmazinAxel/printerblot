import { existsSync, unlinkSync, renameSync, watch } from "node:fs";

const DATA_DIR = process.env.BLOT_DATA_DIR ?? "/var/lib/blotd"; // runtime state
const REPO = process.env.BLOT_REPO ?? "."; // printerblot source (Nix store in prod)
const STATE_FILE = `${DATA_DIR}/state.json`;
const PDF_FILE = `${DATA_DIR}/lastUploadedPDF.pdf`;
const GCODE_FILE = `${DATA_DIR}/lastJob.gcode`;
const PDF2GCODE = process.env.PDF2GCODE ?? "tools/pdf2gcode.py";
const SOCKET_PATH = "/run/blot-socket/blot-socket.sock";

// Send one verb to blotd over its unix socket; resolve with the reply.
// Bun-native socket — no node:net shim. blotd replies then closes the conn.
function daemon(verb: string): Promise<string> {
  return new Promise((resolve) => {
    let buf = "";
    let done = false;
    const finish = (r: string) => { if (!done) { done = true; resolve(r); } };
    Bun.connect({
      unix: SOCKET_PATH,
      socket: {
        open: (s) => { s.write(verb + "\n"); },
        data: (_s, d) => { buf += d.toString(); },
        close: () => finish(buf.trim() || "err"),
        error: () => finish("err"),
      },
    }).catch(() => finish("err"));
  });
}

async function readState(): Promise<any> {
  try {
    return await Bun.file(STATE_FILE).json();
  } catch {
    return { state: "idle", quality: "draft", motorsLocked: false };
  }
}

// Server-Sent Events: push state to the browser only when state.json actually
// changes, so the page needs no polling. blotd writes state.json atomically, so
// watch the directory and react to the rename.
const enc = new TextEncoder();
const sseClients = new Set<ReadableStreamDefaultController>();
let lastBroadcast = "";

async function broadcastState() {
  const s = JSON.stringify(await readState());
  if (s === lastBroadcast) return;
  lastBroadcast = s;
  const chunk = enc.encode(`data: ${s}\n\n`);
  for (const c of sseClients) {
    try { c.enqueue(chunk); } catch {}
  }
}

watch(DATA_DIR, (_event, filename) => {
  if (filename === "state.json") broadcastState();
});

// Save the uploaded PDF and convert it to gcode with the form's settings.
async function handleUpload(req: Request): Promise<Response> {
  const form = await req.formData();
  const file = form.get("pdf");
  if (!(file instanceof File) || file.size === 0) {
    return Response.json({ ok: false, error: "no PDF supplied" }, { status: 400 });
  }

  if (existsSync(PDF_FILE)) unlinkSync(PDF_FILE);
  await Bun.write(PDF_FILE, file);

  const args = [PDF2GCODE, PDF_FILE, "--output-dir", DATA_DIR];
  const num = (k: string, flag: string) => {
    const v = form.get(k);
    if (v != null && String(v).trim() !== "") args.push(flag, String(v));
  };
  num("threshold", "--threshold");
  num("scale", "--scale");
  num("dpi", "--dpi");
  num("margin", "--margin");
  if (form.get("nocrop")) args.push("--no-crop");

  const proc = Bun.spawn(["python3", ...args], {
    stdout: "pipe", stderr: "pipe", cwd: REPO,
  });
  const [out, err, code] = await Promise.all([
    new Response(proc.stdout).text(),
    new Response(proc.stderr).text(),
    proc.exited,
  ]);

  if (code !== 0) {
    return Response.json({ ok: false, error: err || out || "conversion failed" });
  }
  // pdf2gcode names output after the PDF base: lastUploadedPDF.gcode → lastJob.gcode
  const produced = `${DATA_DIR}/lastUploadedPDF.gcode`;
  if (!existsSync(produced)) {
    return Response.json({ ok: false, error: "no gcode produced\n" + out });
  }
  if (existsSync(GCODE_FILE)) unlinkSync(GCODE_FILE);
  renameSync(produced, GCODE_FILE);
  return Response.json({ ok: true });
}

const PAGE = `<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Permablot</title>
</head><body>

<fieldset><legend>Upload</legend>
  <form id="up">
    <input type="file" name="pdf" accept="application/pdf" required>
    <div class="row">
      <label>Threshold <input type="number" name="threshold" value="128" min="0" max="255"></label>
      <label>Scale <input type="number" name="scale" value="1.0" step="0.1" min="0" max="1"></label>
    </div>
    <div class="row">
      <label>DPI <input type="number" name="dpi" value="300" min="50"></label>
      <label>Margin <input type="number" name="margin" value="2" step="0.5" min="0"></label>
      <label><input type="checkbox" name="nocrop"> No crop</label>
    </div>
    <button id="upbtn" type="submit">Upload</button>
  </form>
  <div id="uplog"></div>
</fieldset>

<fieldset><legend>Quality</legend>
  <label><input type="radio" name="q" value="draft"> Draft</label>
  <label><input type="radio" name="q" value="poster"> Poster</label>
</fieldset>

<fieldset><legend>Machine control</legend>
  <span id="status">...</span>
  <div id="controls" class="row" style="margin-top:.6rem"></div>
</fieldset>

<script>
const $ = (s) => document.querySelector(s);

async function post(url, body) {
  const r = await fetch(url, body ? {method: "POST", body} : { method:"POST" });
  return r.json().catch(() => ({}));
}

function render(s) {
  $("#status").textContent =
    s.state === "idle" ? "Not running" : s.state[0].toUpperCase()+s.state.slice(1);

  for (const el of document.getElementsByName("q")) {
    el.checked = el.value === s.quality;
    el.disabled = el.value === s.quality;
  }

  // controls only when idle
  const c = $("#controls");
  if (s.state === "idle") {
    c.innerHTML =
      '<button id="bsleep">Sleep</button>' +
      '<button id="block">' + (s.motorsLocked ? "Unlock motors" : "Lock motors") + '</button>';
    $("#bsleep").onclick = async () => { await post("/sleep"); };
    $("#block").onclick = async () => { await post(s.motorsLocked ? "/unlock" : "/lock"); };
  } else {
    c.innerHTML = "";
  }
}

for (const el of document.getElementsByName("q")) {
  el.addEventListener("change", async () => {
    await post("/quality", new URLSearchParams({q: el.value}));
  });
}

$("#up").addEventListener("submit", async (e) => {
  e.preventDefault();
  $("#upbtn").disabled = true;
  $("#uplog").textContent = "Converting to gcode...";
  const res = await post("/upload", new FormData($("#up")));
  $("#uplog").textContent = res.ok ? "Done!" : "Error: " + (res.error||"failed");
  $("#upbtn").disabled = false;
});

const es = new EventSource("/events");
es.onmessage = (e) => render(JSON.parse(e.data));
</script>
</body></html>`;

Bun.serve({
  port: 80,
  idleTimeout: 255, // keep SSE connections (and long PDF conversions) alive
  async fetch(req) {
    const url = new URL(req.url);
    const { pathname } = url;

    if (pathname === "/" && req.method === "GET") {
      return new Response(PAGE, { headers: { "content-type": "text/html" } });
    }
    if (pathname === "/state" && req.method === "GET") {
      return Response.json(await readState());
    }
    if (pathname === "/events" && req.method === "GET") {
      let controllerRef: ReadableStreamDefaultController;
      let ping: ReturnType<typeof setInterval>;
      const stream = new ReadableStream({
        async start(controller) {
          controllerRef = controller;
          sseClients.add(controller);
          controller.enqueue(enc.encode(`data: ${JSON.stringify(await readState())}\n\n`));
          ping = setInterval(() => {
            try { controller.enqueue(enc.encode(": ping\n\n")); } catch {}
          }, 25000);
        },
        cancel() {
          clearInterval(ping);
          sseClients.delete(controllerRef);
        },
      });
      return new Response(stream, {
        headers: {
          "content-type": "text/event-stream",
          "cache-control": "no-cache",
        },
      });
    }
    if (pathname === "/upload" && req.method === "POST") {
      return handleUpload(req);
    }
    if (pathname === "/quality" && req.method === "POST") {
      const body = await req.formData();
      const q = String(body.get("q"));
      if (q !== "draft" && q !== "poster") {
        return Response.json({ ok: false }, { status: 400 });
      }
      return Response.json({ ok: (await daemon("quality:" + q)) === "ok" });
    }
    if (["/lock", "/unlock", "/sleep"].includes(pathname) && req.method === "POST") {
      const reply = await daemon(pathname.slice(1));
      return Response.json({ ok: reply === "ok" });
    }
    return new Response("not found", { status: 404 });
  }
});
