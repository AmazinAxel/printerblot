import { watch } from "fs"; // no Bun-native file watcher yet

const DATA_DIR = process.env.BLOT_DATA_DIR; // runtime state
const REPO = process.env.BLOT_REPO; // printerblot source
const STATE_FILE = DATA_DIR + 'state.json';
const PDF_FILE = DATA_DIR + '/lastUploadedPDF.pdf';
const GCODE_FILE = DATA_DIR + '/lastJob.gcode';
const PDF2GCODE = process.env.PDF2GCODE;
const SOCKET_PATH = "/run/blot-socket/blot-socket.sock";

// connects with blotd
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
    return { state: "idle", quality: "draft", motorsLocked: true }; // default values if the file is not created yet, for new installs
  };
};

// automatic page updates!
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
  };
};

watch(DATA_DIR, (_event, filename) => {
  if (filename === "state.json") broadcastState();
});

async function handleUpload(req: Request): Promise<Response> {
  const form = await req.formData();
  const file = form.get("pdf");
  if (!(file instanceof File) || file.size === 0) {
    return Response.json({ ok: false, error: "No PDF supplied" }, { status: 400 });
  }

  await Bun.write(PDF_FILE, file); // overwrites any previous upload

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
    return Response.json({ ok: false, error: err || out || "Conversion failed" });
  }

  const produced = Bun.file(DATA_DIR + '/lastUploadedPDF.gcode');
  if (!(await produced.exists()))
    return Response.json({ ok: false, error: "No gcode produced\n" + out });

  await Bun.write(GCODE_FILE, produced); // move into place (overwrites)
  await produced.delete();
  return Response.json({ ok: true });
}

const PAGE = await Bun.file(REPO + '/webpage.html').text();

Bun.serve({
  port: 80,
  idleTimeout: 255, // keep SSE connections and long PDF conversions alive
  async fetch(req) {
    const url = new URL(req.url);
    const { pathname } = url;

    if (pathname === "/" && req.method === "GET")
      return new Response(PAGE, { headers: { "content-type": "text/html" } });
    if (pathname === "/upload" && req.method === "POST")
      return handleUpload(req);
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
        }
      });
      return new Response(stream, {
        headers: {
          "content-type": "text/event-stream",
          "cache-control": "no-cache",
        }
      });
    };
    if (pathname === "/quality" && req.method === "POST") {
      const body = await req.formData();
      const q = String(body.get("q"));
      if (q !== "draft" && q !== "poster") {
        return Response.json({ ok: false }, { status: 400 });
      };
      return Response.json({ ok: (await daemon("quality:" + q)) === "ok" });
    };
    if (["/print", "/lock", "/unlock", "/sleep"].includes(pathname) && req.method === "POST") {
      const reply = await daemon(pathname.slice(1));
      return Response.json({ ok: reply === "ok" });
    };
    return new Response("not found", { status: 404 });
  }
});
