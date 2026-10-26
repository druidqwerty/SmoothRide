#!/usr/bin/env bash
# Builds SmoothRide.jar, assembles dist/SmoothRide (mod folder) and runs the offline simulator.
set -euo pipefail
cd "$(dirname "$0")/.."

JDK="${JDK:-C:/Users/druid/.openclaw/workspace/_ext/jdk/jdk-25.0.4.1+1/bin}"
GAME="${GAME:-G:/Steam/steamapps/common/ProjectZomboid}"
CP="$GAME/projectzomboid.jar;$GAME/ZombieBuddy.jar"
MODVER="${MODVER:-1.0.0}"

rm -rf build/classes build/sim
mkdir -p build/classes build/sim

"$JDK/javac" --release 25 -encoding UTF-8 -Xlint:all,-serial,-processing -cp "$CP" -d build/classes src/com/druid/smoothride/*.java
printf 'Implementation-Title: SmoothRide\nImplementation-Version: %s\n' "$MODVER" > build/manifest.txt
"$JDK/jar" --create --file build/SmoothRide.jar --manifest build/manifest.txt -C build/classes .

# offline simulator: only the game-independent classes
"$JDK/javac" --release 25 -encoding UTF-8 -d build/sim \
  src/com/druid/smoothride/Track.java src/com/druid/smoothride/Sample.java src/com/druid/smoothride/Config.java \
  test/com/druid/smoothride/SimTest.java

rm -rf dist/SmoothRide
mkdir -p dist/SmoothRide/common dist/SmoothRide/42/media/java
cp build/SmoothRide.jar dist/SmoothRide/42/media/java/SmoothRide.jar
cp 42/mod.info dist/SmoothRide/42/mod.info
sed -i "s/^modversion=.*/modversion=$MODVER/" dist/SmoothRide/42/mod.info
[ -f 42/poster.png ] && cp 42/poster.png dist/SmoothRide/42/poster.png
mkdir -p dist/SmoothRide/42/src
cp -r src/com dist/SmoothRide/42/src/

sha256sum build/SmoothRide.jar
if [ "${1:-}" != "--no-sim" ]; then
  "$JDK/java" -cp build/sim com.druid.smoothride.SimTest
fi
