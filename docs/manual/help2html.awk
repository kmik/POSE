# Converts `photogrammetry -h` into the manual's option reference (HTML fragment).
# Section headings: unindented lines ending in ':'. Options: lines indented 2 spaces.
# Descriptions: 6+ spaces. Lines with column gaps (tables) are kept preformatted.
function esc(s,   out, i, c) { out = ""; for (i = 1; i <= length(s); i++) { c = substr(s, i, 1); if (c == "&") c = "&amp;"; else if (c == "<") c = "&lt;"; else if (c == ">") c = "&gt;"; out = out c } return out }
function code(s) { s = esc(s); return s }
function flushpara() { if (para != "") { gsub(/--[a-z0-9][a-z0-9_-]*/, "<span class=\"nw\">&</span>", para); printf "<p>%s</p>\n", para; para = "" } }
function flushpre() { if (pre != "") { printf "<pre class=\"help\">%s</pre>\n", pre; pre = "" } }
function closeitem() { flushpara(); flushpre(); if (initem) { print "</dd>"; initem = 0 } }
function closelist() { closeitem(); if (inlist) { print "</dl>"; inlist = 0 } }
BEGIN { started = 0; nsec = 0 }
/^Examples \(from the repository directory\):/ { stop = 1 }
stop { next }
/^Pipeline steps/ { started = 1 }
!started { next }
/^[A-Z][^ ].*:$/ || /^Orthomosaic \(ORTHOMOSAIC.md\):$/ {
  closelist()
  title = $0; sub(/:$/, "", title); sub(/ \([^)]*\.md[^)]*\)/, "", title)
  nsec++
  printf "<h2 id=\"ref-%d\">A.%d %s</h2>\n", nsec, nsec, esc(title)
  print "<dl class=\"opts\">"; inlist = 1
  next
}
/^  [^ ]/ {
  if (!inlist) { print "<dl class=\"opts\">"; inlist = 1 }
  closeitem()
  line = $0; sub(/^  /, "", line)
  printf "<dt><code>%s</code></dt>\n<dd>", code(line); initem = 1
  next
}
/^ +[^ ]/ {
  line = $0
  if (!initem) { if (!inlist) { print "<dl class=\"opts\">"; inlist = 1 } ; printf "<dt></dt><dd>"; initem = 1 }
  body = line; sub(/^ +/, "", body)
  if (body ~ /[^ ]   +[^ ]/ || line ~ /^        /) { flushpara(); pre = pre (pre == "" ? "" : "\n") esc(body); next }
  flushpre()
  para = para (para == "" ? "" : " ") esc(body)
  next
}
/^$/ { flushpara(); flushpre(); next }
{ closelist(); printf "<p>%s</p>\n", esc($0) }
END { closelist() }
