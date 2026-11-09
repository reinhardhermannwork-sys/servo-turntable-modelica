#!/bin/sh
# Simulate every test model in the OpenModelica container; prints PASS/FAIL, exit 1 if anything failed.
# Needs Docker. Usage: sh test.sh
set -e
TESTS="DriveSequence NoOvershoot"
mkdir -p tout omlib
{ echo 'installPackage(Modelica, "4.1.0+maint.om", exactMatch=true); loadModel(Modelica); loadFile("ServoTurntable/package.mo"); getErrorString(); cd("tout");'
  for t in $TESTS; do
    echo "print(\"### $t\\n\"); res := simulate(ServoTurntable.Tests.$t, outputFormat=\"csv\", fileNamePrefix=\"$t\"); print(res.messages + \"\\n\" + getErrorString() + \"\\n\");"
  done; } > tout/test.mos
docker run --rm -v "$PWD":/w -v "$PWD/omlib":/root/.openmodelica -w /w \
  openmodelica/openmodelica:v1.27.1-ompython omc tout/test.mos > tout/test.log 2>&1 || true
fail=0
for t in $TESTS; do
  block=$(awk -v t="### $t" '$0==t{f=1;next} /^### /{f=0} f' tout/test.log)
  if [ -f "tout/${t}_res.csv" ] && ! echo "$block" | grep -q -E "CHECK FAILED|Error|failed"; then echo "PASS $t"
  else echo "FAIL $t"; echo "$block" | grep -E "CHECK FAILED|Error|failed" | head -5 | sed 's/^/     /'; fail=1; fi
done
exit $fail
