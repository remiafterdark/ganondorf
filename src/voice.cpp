

#include "ganon.hpp"

#include "mods/svc/hook.hpp"

#include "Z2AudioLib/Z2SeMgr.h"
#include "Z2AudioLib/Z2SoundObject.h"
#include "d/actor/d_a_alink.h"
#include "d/d_com_inf_game.h"

DEFINE_HOOK_SYMBOL("Z2SoundObjBase::startSound",
    Z2SoundHandlePool*(Z2SoundObjBase*, JAISoundID, u32, s8), GanonObjSoundHook);
DEFINE_HOOK_SYMBOL("Z2SoundObjBase::startLevelSound",
    Z2SoundHandlePool*(Z2SoundObjBase*, JAISoundID, u32, s8), GanonObjLevelSoundHook);
DEFINE_HOOK_SYMBOL("Z2SeMgr::seStart",
    bool(Z2SeMgr*, JAISoundID, const Vec*, u32, s8, f32, f32, f32, f32, u8), GanonSeStartHook);
DEFINE_HOOK_SYMBOL("Z2SeMgr::seStartLevel",
    bool(Z2SeMgr*, JAISoundID, const Vec*, u32, s8, f32, f32, f32, f32, u8), GanonSeStartLevelHook);
DEFINE_HOOK(&daAlink_c::execute, GanonAlinkExecuteHook);

namespace {

bool s_inExecute = false;

bool is_link_voice(u32 id) {
    return id >= 0x10000u && id <= 0x100C7u;
}

bool inside_link(const daAlink_c* alink, const void* p) {
    const char* c = static_cast<const char*>(p);
    const char* lo = reinterpret_cast<const char*>(alink);
    return c != nullptr && c >= lo && c < lo + sizeof(daAlink_c);
}

bool silence(void* args, bool objectSound) {
    if (!ganon_options().silent || !is_link_voice(mods::arg<JAISoundID>(args, 1))) return false;
    daAlink_c* alink = daAlink_getAlinkActorClass();
    if (alink == nullptr || !ganon_is_him(alink)) return false;
    const void* at = objectSound ? static_cast<const void*>(mods::arg<Z2SoundObjBase*>(args, 0))
                                 : static_cast<const void*>(mods::arg<const Vec*>(args, 2));
    return s_inExecute || inside_link(alink, at);
}

HookAction on_obj_sound_pre(ModContext*, void* args, void* ret, void*) {
    if (!silence(args, true)) return HOOK_CONTINUE;
    if (ret != nullptr) *static_cast<Z2SoundHandlePool**>(ret) = nullptr;
    return HOOK_SKIP_ORIGINAL;
}

HookAction on_se_pre(ModContext*, void* args, void* ret, void*) {
    if (!silence(args, false)) return HOOK_CONTINUE;
    if (ret != nullptr) *static_cast<bool*>(ret) = false;
    return HOOK_SKIP_ORIGINAL;
}

}

void voice_init() {
    const int results[] = {
        static_cast<int>(mods::hook::add_pre<GanonObjSoundHook>(on_obj_sound_pre)),
        static_cast<int>(mods::hook::add_pre<GanonObjLevelSoundHook>(on_obj_sound_pre)),
        static_cast<int>(mods::hook::add_pre<GanonSeStartHook>(on_se_pre)),
        static_cast<int>(mods::hook::add_pre<GanonSeStartLevelHook>(on_se_pre)),
        static_cast<int>(mods::hook::add_pre<GanonAlinkExecuteHook>(
            [](ModContext*, void*, void*, void*) -> HookAction {
                s_inExecute = true;
                return HOOK_CONTINUE;
            })),
        static_cast<int>(mods::hook::add_post<GanonAlinkExecuteHook>(
            [](ModContext*, void*, void*, void*) { s_inExecute = false; })),
    };
    int failed = 0;
    for (int r : results) failed += r != 0 ? 1 : 0;
    mods::log::info("ganondorf: {} of 6 voice hooks in", 6 - failed);
}
