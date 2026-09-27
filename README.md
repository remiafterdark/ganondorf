# Ganondorf

Play as Ganondorf in Twilight Princess, as a mod for [Dusklight](https://github.com/TwilitRealm/dusklight).

Ganondorf from the final battle over Link, eyes and all, with his sword in place of yours.
Everything comes from the game itself, nothing is replaced on disk.

## Install

Needs Dusklight 2.0.2 or newer. Put `ganondorf.dusk` in Dusklight's `mods` folder.

## Options

In the Mods window, under Ganondorf. Everything is on by default.

| Option | What it does |
| --- | --- |
| Be Ganondorf | his body in place of Link's |
| In every outfit | or only the outfits you pick |
| Silent | no voice while you are him |
| His sword | over the Wooden, Ordon and/or Master Sword |

## Co-op

Only your own screen changes. If you play [Crests of Courage](https://github.com/remiafterdark/crests-of-courage),
pick Ganondorf in its Models tab instead so everyone sees him, and don't use both at once.

## Building

```
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

The Dusklight source is fetched on the first configure, or put a checkout next to this folder.
The mod ends up in `build/mods/ganondorf.dusk`.
