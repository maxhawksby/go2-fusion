import { createReadStream } from 'node:fs';
import { promises as fs } from 'node:fs';
import { createServer } from 'node:http';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';


const MODULE_DIR = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(MODULE_DIR, '..', '..');
const PUBLIC_DIR = path.join(MODULE_DIR, 'public');
const DEFAULT_RESULTS_ROOT = path.join(REPO_ROOT, 'src', 'go2_scoreboard', 'results');
const SKIPPED_DIRECTORIES = new Set([
  '.git', '.pytest_cache', '__pycache__', 'bag', 'build', 'install', 'log', 'logs', 'maps',
]);

const MIME_TYPES = new Map([
  ['.css', 'text/css; charset=utf-8'],
  ['.html', 'text/html; charset=utf-8'],
  ['.js', 'text/javascript; charset=utf-8'],
  ['.mjs', 'text/javascript; charset=utf-8'],
  ['.json', 'application/json; charset=utf-8'],
  ['.svg', 'image/svg+xml'],
]);


function isInside(root, candidate) {
  const relative = path.relative(root, candidate);
  return relative === '' || (!relative.startsWith('..') && !path.isAbsolute(relative));
}


export function resolveRunDirectory(resultsRoot, relativePath) {
  if (!relativePath || path.isAbsolute(relativePath)) {
    throw new Error('A relative run path is required.');
  }
  const root = path.resolve(resultsRoot);
  const candidate = path.resolve(root, relativePath);
  if (!isInside(root, candidate)) {
    throw new Error('Run path escapes the configured results directory.');
  }
  return candidate;
}


export function parseCsv(text) {
  const rows = [];
  let row = [];
  let cell = '';
  let quoted = false;

  for (let index = 0; index < text.length; index += 1) {
    const character = text[index];
    if (quoted) {
      if (character === '"' && text[index + 1] === '"') {
        cell += '"';
        index += 1;
      } else if (character === '"') {
        quoted = false;
      } else {
        cell += character;
      }
    } else if (character === '"') {
      quoted = true;
    } else if (character === ',') {
      row.push(cell);
      cell = '';
    } else if (character === '\n') {
      row.push(cell.replace(/\r$/, ''));
      rows.push(row);
      row = [];
      cell = '';
    } else {
      cell += character;
    }
  }
  if (cell || row.length) {
    row.push(cell.replace(/\r$/, ''));
    rows.push(row);
  }
  if (rows.length < 2) {
    return [];
  }
  const headers = rows.shift();
  return rows
    .filter((values) => values.some((value) => value !== ''))
    .map((values) => Object.fromEntries(
      headers.map((header, index) => [header, values[index] ?? '']),
    ));
}


async function readJson(filePath) {
  return JSON.parse(await fs.readFile(filePath, 'utf8'));
}


async function optionalText(filePath) {
  try {
    return await fs.readFile(filePath, 'utf8');
  } catch (error) {
    if (error.code === 'ENOENT') {
      return '';
    }
    throw error;
  }
}


async function optionalFile(filePath) {
  try {
    await fs.access(filePath);
    return true;
  } catch {
    return false;
  }
}


async function findRunFiles(directory, depth, found) {
  if (depth < 0) {
    return;
  }
  let entries;
  try {
    entries = await fs.readdir(directory, { withFileTypes: true });
  } catch (error) {
    if (error.code === 'ENOENT') {
      return;
    }
    throw error;
  }
  if (entries.some((entry) => entry.isFile() && entry.name === 'run.json')) {
    found.push(path.join(directory, 'run.json'));
    return;
  }
  await Promise.all(entries
    .filter((entry) => entry.isDirectory() && !SKIPPED_DIRECTORIES.has(entry.name))
    .map((entry) => findRunFiles(path.join(directory, entry.name), depth - 1, found)));
}


export async function discoverRuns(resultsRoot, maxDepth = 8) {
  const root = path.resolve(resultsRoot);
  const runFiles = [];
  await findRunFiles(root, maxDepth, runFiles);
  const runs = [];
  for (const runFile of runFiles) {
    try {
      const [summary, stats] = await Promise.all([readJson(runFile), fs.stat(runFile)]);
      const directory = path.dirname(runFile);
      runs.push({
        ...summary,
        path: path.relative(root, directory).split(path.sep).join('/'),
        updated_at: stats.mtime.toISOString(),
        has_map: await optionalFile(path.join(directory, 'final_map.pgm')),
        has_timeline: await optionalFile(path.join(directory, 'timeline.csv')),
      });
    } catch (error) {
      if (error.name !== 'SyntaxError' && error.code !== 'ENOENT') {
        throw error;
      }
    }
  }
  return runs.sort((left, right) => right.updated_at.localeCompare(left.updated_at));
}


export async function loadRun(resultsRoot, relativePath) {
  const directory = resolveRunDirectory(resultsRoot, relativePath);
  const [summary, timelineText, resourcesText] = await Promise.all([
    readJson(path.join(directory, 'run.json')),
    optionalText(path.join(directory, 'timeline.csv')),
    optionalText(path.join(directory, 'resources.csv')),
  ]);
  return {
    summary,
    timeline: parseCsv(timelineText),
    resources: parseCsv(resourcesText),
    files: {
      map: await optionalFile(path.join(directory, 'final_map.pgm')),
      report: await optionalFile(path.join(directory, 'report.md')),
      scorecard: await optionalFile(path.join(directory, 'scorecard.csv')),
    },
  };
}


function jsonResponse(response, status, body) {
  response.writeHead(status, {
    'Cache-Control': 'no-store',
    'Content-Type': 'application/json; charset=utf-8',
    'X-Content-Type-Options': 'nosniff',
  });
  response.end(`${JSON.stringify(body)}\n`);
}


function errorResponse(response, status, message) {
  jsonResponse(response, status, { error: message });
}


async function sendFile(response, filePath, contentType, downloadName = '') {
  const stats = await fs.stat(filePath);
  const headers = {
    'Cache-Control': 'no-store',
    'Content-Length': stats.size,
    'Content-Type': contentType,
    'X-Content-Type-Options': 'nosniff',
  };
  if (downloadName) {
    headers['Content-Disposition'] = `attachment; filename="${downloadName}"`;
  }
  response.writeHead(200, headers);
  createReadStream(filePath).pipe(response);
}


async function serveApi(requestUrl, response, resultsRoot) {
  if (requestUrl.pathname === '/api/runs') {
    jsonResponse(response, 200, { runs: await discoverRuns(resultsRoot) });
    return true;
  }
  if (requestUrl.pathname === '/api/run') {
    const runPath = requestUrl.searchParams.get('path');
    jsonResponse(response, 200, await loadRun(resultsRoot, runPath));
    return true;
  }
  if (requestUrl.pathname === '/api/map' || requestUrl.pathname === '/api/download') {
    const runPath = requestUrl.searchParams.get('path');
    const directory = resolveRunDirectory(resultsRoot, runPath);
    const requestedFile = requestUrl.pathname === '/api/map'
      ? 'final_map.pgm'
      : requestUrl.searchParams.get('file');
    const allowed = new Map([
      ['final_map.pgm', 'image/x-portable-graymap'],
      ['report.md', 'text/markdown; charset=utf-8'],
      ['run.json', 'application/json; charset=utf-8'],
      ['scorecard.csv', 'text/csv; charset=utf-8'],
      ['timeline.csv', 'text/csv; charset=utf-8'],
    ]);
    if (!allowed.has(requestedFile)) {
      throw new Error('That artifact is not available for download.');
    }
    await sendFile(
      response,
      path.join(directory, requestedFile),
      allowed.get(requestedFile),
      requestUrl.pathname === '/api/download' ? requestedFile : '',
    );
    return true;
  }
  return false;
}


async function servePublic(requestUrl, response) {
  const requested = requestUrl.pathname === '/' ? 'index.html' : requestUrl.pathname.slice(1);
  const filePath = path.resolve(PUBLIC_DIR, requested);
  if (!isInside(PUBLIC_DIR, filePath)) {
    errorResponse(response, 403, 'Invalid asset path.');
    return;
  }
  await sendFile(
    response,
    filePath,
    MIME_TYPES.get(path.extname(filePath)) || 'application/octet-stream',
  );
}


export function createDashboardServer({ resultsRoot = DEFAULT_RESULTS_ROOT } = {}) {
  const root = path.resolve(resultsRoot);
  return createServer(async (request, response) => {
    if (request.method !== 'GET') {
      errorResponse(response, 405, 'The scoreboard dashboard is read-only.');
      return;
    }
    try {
      const requestUrl = new URL(request.url, 'http://localhost');
      if (requestUrl.pathname.startsWith('/api/')) {
        if (!await serveApi(requestUrl, response, root)) {
          errorResponse(response, 404, 'Unknown API route.');
        }
        return;
      }
      await servePublic(requestUrl, response);
    } catch (error) {
      const status = error.code === 'ENOENT' ? 404 : 400;
      errorResponse(response, status, error.message || 'Unable to load scoreboard data.');
    }
  });
}


function parseArguments(argumentsList) {
  const options = {
    host: '127.0.0.1',
    port: 3100,
    resultsRoot: DEFAULT_RESULTS_ROOT,
  };
  for (let index = 0; index < argumentsList.length; index += 1) {
    const argument = argumentsList[index];
    if (argument === '--host') {
      options.host = argumentsList[index += 1];
    } else if (argument === '--port') {
      options.port = Number(argumentsList[index += 1]);
    } else if (argument === '--results-root') {
      options.resultsRoot = path.resolve(argumentsList[index += 1]);
    } else if (argument === '--help') {
      options.help = true;
    } else {
      throw new Error(`Unknown argument: ${argument}`);
    }
  }
  if (!Number.isInteger(options.port) || options.port < 1 || options.port > 65535) {
    throw new Error('Port must be an integer from 1 to 65535.');
  }
  return options;
}


async function main() {
  const options = parseArguments(process.argv.slice(2));
  if (options.help) {
    process.stdout.write(
      'Usage: node host_tools/scoreboard_dashboard/server.mjs '
      + '[--port 3100] [--host 127.0.0.1] [--results-root PATH]\n',
    );
    return;
  }
  const server = createDashboardServer(options);
  server.listen(options.port, options.host, () => {
    process.stdout.write(`Oak Mission Scoreboard: http://${options.host}:${options.port}\n`);
    process.stdout.write(`Reading runs from: ${path.resolve(options.resultsRoot)}\n`);
  });
}


if (import.meta.url === pathToFileURL(process.argv[1] || '').href) {
  main().catch((error) => {
    process.stderr.write(`${error.message}\n`);
    process.exitCode = 1;
  });
}
