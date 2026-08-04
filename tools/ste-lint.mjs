#!/usr/bin/env node
/**
 * ste-lint checks Markdown prose against the writing rules in
 * docs/CONVENTIONS.md section 1.
 *
 * Usage: node tools/ste-lint.mjs <path> [<path>...] [--quiet]
 *
 * A path is a file or a directory. The tool walks a directory and checks every
 * .md file in it.
 *
 * The tool reads prose only. It masks fenced code, indented code, inline code
 * spans, link targets, HTML comments, and table rows that hold pure data. A
 * masked region becomes spaces, so line numbers and column numbers stay true.
 *
 * Exit code 1 when an error level rule fires. PASSIVE is warning level, because
 * some passive sentences are correct. A warning never fails the build.
 */

import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join, extname, relative, basename } from 'node:path';

/* ------------------------------------------------------------------ rules */

const MAX_SENTENCE_WORDS = 25;

const CONTRACTIONS = [
  "don't", "doesn't", "isn't", "aren't", "won't", "can't", "it's", "that's",
  "there's", "we're", "you're", "they're", "wasn't", "weren't", "hasn't",
  "haven't", "didn't", "couldn't", "shouldn't", "wouldn't", "let's", "I'm",
];

/** Each entry is [regular expression source, plain replacement]. */
const WORDY = [
  ['utiliz(?:e|es|ed|ing|ation)', 'use'],
  ['leverag(?:e|es|ed|ing)', 'use'],
  ['facilitat(?:e|es|ed|ing|ion)', 'help'],
  ['ensur(?:e|es|ed|ing)', 'make sure'],
  ['prior\\s+to', 'before'],
  ['subsequent\\s+to', 'after'],
  ['regarding', 'about'],
  ['concerning', 'about'],
  ['obtain(?:s|ed|ing)?', 'get'],
  ['acquir(?:e|es|ed|ing)', 'get'],
  ['demonstrat(?:e|es|ed|ing|ion)', 'show'],
  ['additionally', 'also'],
  ['furthermore', 'also'],
  ['moreover', 'also'],
  ['in\\s+order\\s+to', 'to'],
  ['a\\s+number\\s+of', 'several'],
  ['at\\s+this\\s+point\\s+in\\s+time', 'now'],
  ['due\\s+to\\s+the\\s+fact\\s+that', 'because'],
  ['it\\s+should\\s+be\\s+noted\\s+that', '(delete)'],
];

const MARKETING = [
  'seamless', 'robust', 'powerful', 'cutting-edge', 'effortless', 'world-class',
  'next-generation', 'revolutionary', 'blazing', 'delightful', 'elegant',
  'comprehensive',
];

const BE_FORMS = new Set(['is', 'are', 'was', 'were', 'been', 'being']);

const IRREGULAR_PARTICIPLES = new Set([
  'made', 'done', 'given', 'taken', 'written', 'built', 'held', 'run', 'set',
  'shown', 'known', 'found', 'seen', 'kept',
]);

/** Words that end in "ed" and are not past participles. */
const NOT_PARTICIPLES = new Set([
  'ahead', 'instead', 'spread', 'thread', 'bread', 'dead', 'head', 'read',
  'hundred', 'sacred', 'embed', 'shed', 'sled', 'bled', 'fled', 'naked',
  'wicked', 'inbred', 'moped', 'oped',
]);

/** Abbreviations that end in a period and do not end a sentence. */
const ABBREVIATIONS = [
  'e.g.', 'i.e.', 'etc.', 'vs.', 'cf.', 'fig.', 'no.', 'approx.', 'dr.', 'mr.',
  'ms.', 'st.', 'al.', 'ca.',
];

const SEVERITY = {
  LENGTH: 'error',
  SEMICOLON: 'error',
  CONTRACTION: 'error',
  WORDY: 'error',
  MARKETING: 'error',
  PASSIVE: 'warning',
  EMOJI: 'error',
};

/* ------------------------------------------------------- compiled patterns */

const APOSTROPHE = "['’]";

const CONTRACTION_RE = new RegExp(
  '\\b(?:' + CONTRACTIONS.map((c) => escape(c).replace("'", APOSTROPHE)).join('|') + ')\\b',
  'gi',
);

const WORDY_RES = WORDY.map(([source, plain]) => [
  new RegExp('\\b(?:' + source + ')\\b', 'gi'),
  plain,
]);

const MARKETING_RE = new RegExp(
  '\\b(?:' + MARKETING.map(escape).join('|') + ')(?:ly|ness|s)?\\b',
  'gi',
);

const EMOJI_RE = new RegExp(
  '['
  + '\\u{1F000}-\\u{1FAFF}'
  + '\\u{1F1E6}-\\u{1F1FF}'
  + '\\u{2600}-\\u{27BF}'
  + '\\u{2B00}-\\u{2BFF}'
  + '\\u{FE0F}\\u{20E3}'
  + '\\u{203C}\\u{2049}\\u{2122}\\u{2139}'
  + '\\u{2194}-\\u{21AA}'
  + '\\u{231A}-\\u{231B}\\u{2328}\\u{23CF}'
  + '\\u{23E9}-\\u{23FA}'
  + '\\u{24C2}'
  + '\\u{25AA}-\\u{25FE}'
  + '\\u{2934}\\u{2935}'
  + '\\u{3030}\\u{303D}\\u{3297}\\u{3299}'
  + ']',
  'gu',
);

function escape(text) {
  return text.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

function blank(text) {
  return text.replace(/[^\n]/g, ' ');
}

/* -------------------------------------------------------------- file walk */

const SKIP_DIRS = new Set(['node_modules', '.git', 'dist', 'build', 'coverage']);

function collect(path, found) {
  const info = statSync(path);
  if (info.isDirectory()) {
    if (SKIP_DIRS.has(basename(path))) return found;
    for (const entry of readdirSync(path).sort()) {
      if (entry.startsWith('.')) continue;
      collect(join(path, entry), found);
    }
    return found;
  }
  if (extname(path).toLowerCase() === '.md') found.push(path);
  return found;
}

/* ---------------------------------------------------------------- masking */

/** Replaces every inline code span with spaces of the same width. */
function maskCodeSpans(line) {
  let out = '';
  let i = 0;
  while (i < line.length) {
    if (line[i] !== '`') {
      out += line[i];
      i += 1;
      continue;
    }
    let open = 0;
    while (line[i + open] === '`') open += 1;
    let j = i + open;
    let close = -1;
    while (j < line.length) {
      if (line[j] === '`') {
        let run = 0;
        while (line[j + run] === '`') run += 1;
        if (run === open) {
          close = j;
          break;
        }
        j += run;
      } else {
        j += 1;
      }
    }
    if (close === -1) {
      out += line.slice(i, i + open);
      i += open;
    } else {
      out += ' '.repeat(close + open - i);
      i = close + open;
    }
  }
  return out;
}

/** True when a table row holds pure data, that is, no cell ends a sentence. */
function isDataRow(line) {
  if (!/^\s{0,3}\|/.test(line)) return false;
  const cells = line.split('|');
  for (const cell of cells) {
    if (/\.(\s|$)/.test(cell.trim())) return false;
  }
  return true;
}

/**
 * Returns one masked line per source line. Every non prose region holds spaces.
 */
function maskLines(text) {
  const withoutComments = text.replace(/<!--[\s\S]*?-->/g, blank);
  const source = withoutComments.split(/\r?\n/);
  const masked = [];

  let fence = null;
  let previousBlank = true;
  let inIndentedCode = false;

  for (const line of source) {
    const fenceMatch = /^\s{0,3}(`{3,}|~{3,})/.exec(line);

    if (fence !== null) {
      masked.push(blank(line));
      if (fenceMatch && fenceMatch[1][0] === fence.char && fenceMatch[1].length >= fence.length) {
        fence = null;
      }
      previousBlank = false;
      continue;
    }
    if (fenceMatch) {
      fence = { char: fenceMatch[1][0], length: fenceMatch[1].length };
      masked.push(blank(line));
      previousBlank = false;
      continue;
    }

    const isBlank = line.trim() === '';
    const isIndented = /^(?: {4,}|\t)/.test(line) && !isBlank;

    if (inIndentedCode && (isIndented || isBlank)) {
      masked.push(blank(line));
      previousBlank = isBlank;
      if (!isBlank) previousBlank = false;
      continue;
    }
    inIndentedCode = false;
    if (previousBlank && isIndented) {
      inIndentedCode = true;
      masked.push(blank(line));
      previousBlank = false;
      continue;
    }

    let out = maskCodeSpans(line);
    if (/^\s{0,3}\[[^\]]+\]:\s*\S+/.test(out)) out = blank(out);
    out = out.replace(/\]\([^)]*\)/g, (m) => ']' + ' '.repeat(m.length - 1));
    out = out.replace(/<[^<>\s][^<>]*>/g, blank);
    out = out.replace(/\bhttps?:\/\/\S+/g, blank);
    if (isDataRow(out)) out = blank(out);

    masked.push(out);
    previousBlank = isBlank;
  }
  return masked;
}

/* ----------------------------------------------------------- block layout */

/**
 * Groups masked lines into prose blocks. A block is a heading, a list item, or
 * a paragraph, with its wrapped continuation lines joined.
 */
function buildBlocks(masked) {
  const blocks = [];
  let current = null;

  const push = (line, text, ownBlock) => {
    if (ownBlock || current === null) {
      current = { parts: [] };
      blocks.push(current);
    }
    current.parts.push({ line, text });
  };

  for (let i = 0; i < masked.length; i += 1) {
    const line = masked[i];
    const number = i + 1;
    if (line.trim() === '') {
      current = null;
      continue;
    }
    if (/^\s{0,3}(?:-{3,}|\*{3,}|_{3,}|={3,})\s*$/.test(line)) {
      current = null;
      continue;
    }
    let text = line;
    let ownBlock = false;

    const heading = /^\s{0,3}#{1,6}\s+/.exec(text);
    if (heading) {
      text = ' '.repeat(heading[0].length) + text.slice(heading[0].length);
      ownBlock = true;
      current = null;
    }
    const item = /^\s*(?:[-*+]|\d+[.)])\s+/.exec(text);
    if (!heading && item) {
      text = ' '.repeat(item[0].length) + text.slice(item[0].length);
      ownBlock = true;
      current = null;
    }
    const quote = /^\s{0,3}>\s?/.exec(text);
    if (quote) text = ' '.repeat(quote[0].length) + text.slice(quote[0].length);

    push(number, text, ownBlock);
    if (heading) current = null;
  }

  return blocks.map((block) => {
    let full = '';
    const offsets = [];
    for (const part of block.parts) {
      if (full.length > 0) full += ' ';
      offsets.push({ start: full.length, line: part.line });
      full += part.text.trim();
    }
    return { text: full, offsets };
  }).filter((block) => block.text.trim() !== '');
}

function lineAt(block, position) {
  let line = block.offsets[0].line;
  for (const offset of block.offsets) {
    if (offset.start <= position) line = offset.line;
    else break;
  }
  return line;
}

/* -------------------------------------------------------------- sentences */

function splitSentences(text) {
  const sentences = [];
  const boundary = /[.!?](?=["'’)\]`*]*(?:\s|$))/g;
  let start = 0;
  let match;
  while ((match = boundary.exec(text)) !== null) {
    const end = match.index + 1;
    const tail = text.slice(Math.max(0, end - 8), end).toLowerCase();
    if (ABBREVIATIONS.some((a) => tail.endsWith(a))) continue;
    const piece = text.slice(start, end);
    if (piece.trim() !== '') sentences.push({ start, text: piece });
    let next = end;
    while (next < text.length && /[\s"'’)\]`*]/.test(text[next])) next += 1;
    start = next;
    boundary.lastIndex = next;
  }
  if (start < text.length && text.slice(start).trim() !== '') {
    sentences.push({ start, text: text.slice(start) });
  }
  return sentences;
}

function countWords(text) {
  return text.split(/\s+/).filter((token) => /[A-Za-z0-9]/.test(token)).length;
}

function isParticiple(word) {
  if (IRREGULAR_PARTICIPLES.has(word)) return true;
  if (!word.endsWith('ed')) return false;
  if (word.length < 4) return false;
  if (word.endsWith('eed')) return false;
  return !NOT_PARTICIPLES.has(word);
}

/* ----------------------------------------------------------------- checks */

function checkFile(path, label) {
  const findings = [];
  const text = readFileSync(path, 'utf8');
  const masked = maskLines(text);

  const report = (line, rule, message) => {
    findings.push({ file: label, line, rule, message, severity: SEVERITY[rule] });
  };

  // Line level checks.
  masked.forEach((line, index) => {
    const number = index + 1;
    if (line.trim() === '') return;

    if (line.includes(';')) {
      report(number, 'SEMICOLON', 'semicolon in prose, write two sentences');
    }

    for (const match of line.matchAll(CONTRACTION_RE)) {
      report(number, 'CONTRACTION', `contraction "${match[0]}", write the full form`);
    }

    for (const [expression, plain] of WORDY_RES) {
      expression.lastIndex = 0;
      for (const match of line.matchAll(expression)) {
        report(number, 'WORDY', `"${match[0]}" -> ${plain}`);
      }
    }

    for (const match of line.matchAll(MARKETING_RE)) {
      report(number, 'MARKETING', `marketing word "${match[0]}", state a fact instead`);
    }

    for (const match of line.matchAll(EMOJI_RE)) {
      const point = match[0].codePointAt(0).toString(16).toUpperCase();
      report(number, 'EMOJI', `emoji U+${point}, remove it`);
    }
  });

  // Sentence level checks.
  for (const block of buildBlocks(masked)) {
    for (const sentence of splitSentences(block.text)) {
      const words = countWords(sentence.text);
      const at = lineAt(block, block.offsets[0].start + sentence.start);
      if (words > MAX_SENTENCE_WORDS) {
        report(at, 'LENGTH', `sentence has ${words} words, limit ${MAX_SENTENCE_WORDS}: ${preview(sentence.text)}`);
      }
      const tokens = [];
      const wordRe = /[A-Za-z][A-Za-z'’-]*/g;
      let match;
      while ((match = wordRe.exec(sentence.text)) !== null) {
        tokens.push({ word: match[0].toLowerCase(), index: match.index });
      }
      for (let i = 0; i < tokens.length; i += 1) {
        if (!BE_FORMS.has(tokens[i].word)) continue;
        for (let j = i + 1; j <= i + 2 && j < tokens.length; j += 1) {
          if (!isParticiple(tokens[j].word)) continue;
          const phrase = tokens.slice(i, j + 1).map((t) => t.word).join(' ');
          const line = lineAt(block, sentence.start + tokens[i].index);
          report(line, 'PASSIVE', `possible passive voice "${phrase}", prefer active voice`);
          break;
        }
      }
    }
  }

  findings.sort((a, b) => a.line - b.line);
  return findings;
}

function preview(text) {
  const flat = text.trim().replace(/\s+/g, ' ');
  return flat.length <= 60 ? `"${flat}"` : `"${flat.slice(0, 57)}..."`;
}

/* ------------------------------------------------------------------- main */

function main(argv) {
  const quiet = argv.includes('--quiet');
  const paths = argv.filter((arg) => !arg.startsWith('--'));

  if (paths.length === 0) {
    process.stderr.write('usage: node tools/ste-lint.mjs <path> [<path>...] [--quiet]\n');
    return 2;
  }

  const files = [];
  for (const path of paths) {
    try {
      collect(path, files);
    } catch {
      process.stderr.write(`ste-lint: cannot read ${path}\n`);
      return 2;
    }
  }

  const counts = {};
  let errors = 0;
  let warnings = 0;

  for (const file of files.sort()) {
    const label = relative(process.cwd(), file) || file;
    for (const finding of checkFile(file, label)) {
      counts[finding.rule] = (counts[finding.rule] || 0) + 1;
      if (finding.severity === 'error') errors += 1;
      else warnings += 1;
      if (!quiet) {
        process.stdout.write(`${finding.file}:${finding.line}: ${finding.rule} ${finding.message}\n`);
      }
    }
  }

  const rules = Object.keys(counts).sort();
  if (rules.length > 0 && !quiet) process.stdout.write('\n');
  for (const rule of rules) {
    process.stdout.write(`  ${rule}: ${counts[rule]} (${SEVERITY[rule]})\n`);
  }
  process.stdout.write(
    `ste-lint: ${files.length} file(s), ${errors} error(s), ${warnings} warning(s)\n`,
  );

  return errors > 0 ? 1 : 0;
}

process.exit(main(process.argv.slice(2)));
