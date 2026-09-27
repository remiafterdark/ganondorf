#pragma once

#include "mods/api.h"
#include "mods/svc/config.h"
#include "mods/svc/log.hpp"

#include "dolphin/types.h"

#include <cstdint>

class daAlink_c;
class J3DModel;
class J3DModelData;
class JKRHeap;

extern const ConfigService* svc_config;

struct GanonOptions {
    bool body;
    bool allOutfits;
    bool outfit[4];
    bool silent;
    bool sword;
    bool swordFor[3];
};

enum { kOutfitHero, kOutfitOrdon, kOutfitZora, kOutfitMagic };
enum { kSwordWood, kSwordOrdon, kSwordMaster };

const GanonOptions& ganon_options();

bool arc_request(const char* name);
int arc_poll(const char* name);
J3DModelData* arc_load(const char* name, const char* file);
void* arc_load_anm(const char* name, const char* file);
void arc_free_data(J3DModelData* data);
JKRHeap* arc_heap_if_any();
J3DModel* ganon_create_model(J3DModelData* data, u32 modelFlag, u32 differedDlistFlag);
void ganon_free_model(J3DModel* model);

void ganon_init();
void ganon_frame(daAlink_c* alink);

bool ganon_is_him(daAlink_c* alink);

void voice_init();
