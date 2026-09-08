#!/usr/bin/env bash
# Depots tiers construits avec le workspace, epingles a un commit. Clone
# dans <workspace>/src a cote d'alidade_ros2, ou remet a jour un clone
# existant sur le meme commit. Appele par install.sh sur le Pi et par
# alidade_viz/docker/run.sh up dans le conteneur.
#
#   ./system/sources.sh [workspace]     (defaut : le parent de src/alidade_ros2)
#
# Pour changer de version : modifier le commit ici, relancer, recompiler.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$(cd "$HERE/../../.." && pwd)}"
SRC="$WORKSPACE/src"

# nom | depot | branche | commit
SOURCES='
vizanti | https://github.com/MoffKalast/vizanti.git | ros2 | 892f95171c259abdb09d8079c83021a830daf565
'

mkdir -p "$SRC"
echo "$SOURCES" | grep -vE '^\s*(#|$)' | while IFS='|' read -r name url branch commit; do
  name="$(echo "$name" | xargs)"; url="$(echo "$url" | xargs)"
  branch="$(echo "$branch" | xargs)"; commit="$(echo "$commit" | xargs)"
  dest="$SRC/$name"
  if [ ! -d "$dest/.git" ]; then
    echo "== $name : clone $url ($branch)"
    git clone -q --branch "$branch" "$url" "$dest"
  else
    echo "== $name : mise a jour"
    git -C "$dest" fetch -q origin "$branch"
  fi
  git -C "$dest" checkout -q "$commit"
  echo "   $name @ $(git -C "$dest" rev-parse --short HEAD)"
done

# Widgets Alidade dans Vizanti : le serveur decouvre les templates dans son
# propre dossier public (celui des sources avec --symlink-install) par un
# os.walk qui ne suit pas les liens de dossiers ; on cree donc un vrai dossier
# par widget et on y lie chaque fichier.
if [ -d "$SRC/vizanti/vizanti_server/public/templates" ]; then
  for widget in "$HERE/../alidade_ui/templates"/*/; do
    [ -d "$widget" ] || continue
    name="$(basename "$widget")"
    dest="$SRC/vizanti/vizanti_server/public/templates/$name"
    mkdir -p "$dest"
    for file in "$widget"/*; do
      ln -sfn "$(cd "$(dirname "$file")" && pwd)/$(basename "$file")" "$dest/$(basename "$file")"
    done
    echo "   widget $name -> vizanti ($(ls "$dest" | wc -l) fichiers)"
  done
fi
