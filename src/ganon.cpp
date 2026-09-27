

#include "ganon.hpp"

#include "mods/svc/hook.hpp"

#include "d/actor/d_a_alink.h"
#include "d/actor/d_a_mirror.h"
#include "d/d_com_inf_game.h"
#include "d/d_kankyo.h"
#include "f_op/f_op_actor_mng.h"
#include "m_Do/m_Do_ext.h"
#include "JSystem/J3DGraphAnimator/J3DJoint.h"
#include "JSystem/J3DGraphAnimator/J3DModel.h"
#include "JSystem/J3DGraphAnimator/J3DModelData.h"
#include "JSystem/J3DGraphBase/J3DSys.h"
#include "JSystem/J3DGraphBase/J3DTransform.h"
#include "JSystem/JKernel/JKRHeap.h"
#include "JSystem/JUtility/JUTNameTab.h"

#include <cstdlib>
#include <cstring>

DEFINE_HOOK_SYMBOL("daAlink_c::modelDraw", void(daAlink_c*, J3DModel*, int), GanonModelDrawHook);
DEFINE_HOOK_SYMBOL("daAlink_c::setItemMatrix", void(daAlink_c*, int), GanonItemMatrixHook);
DEFINE_HOOK_SYMBOL("daAlink_c::shadowDraw", void(daAlink_c*), GanonShadowHook);
DEFINE_HOOK(&daAlink_c::draw, GanonAlinkDrawHook);

namespace {

const char kArc[] = "B_gnd";
const char kBody[] = "egnd.bmd";
const int kMaxJoints = 64;
const f32 kBackDepth = 1.8f;
const f32 kHeadScale = 1.2f;

struct Finger {
    const char* name;
    f32 closeDegrees;
    bool right;
};
const Finger kFingers[4] = {{"fingerL1", -72.0f, false}, {"fingerL2", -85.0f, false},
    {"fingerR1", -72.0f, true}, {"fingerR2", -85.0f, true}};

enum ArcState { kArcNone, kArcMounting, kArcReady, kArcFailed };
ArcState s_arc = kArcNone;

bool arc_ready() {
    if (s_arc == kArcReady) return true;
    if (s_arc == kArcFailed) return false;
    if (s_arc == kArcNone) s_arc = arc_request(kArc) ? kArcMounting : kArcFailed;
    if (s_arc == kArcFailed) return false;
    const int ready = arc_poll(kArc);
    if (ready == 0) return false;
    s_arc = ready > 0 ? kArcReady : kArcFailed;
    return s_arc == kArcReady;
}

bool s_learned = false;
int s_count = 0;
int s_parent[kMaxJoints];
int s_order[kMaxJoints];
Mtx s_restWorld[kMaxJoints];
Mtx s_restLocal[kMaxJoints];
S16Vec s_restAngle[kMaxJoints];
Vec s_offset[kMaxJoints];
f32 s_rootHeight = 1.0f;
int s_fingerJoint[4] = {-1, -1, -1, -1};
bool s_inHead[kMaxJoints] = {};

void rotation_only(const Mtx in, Mtx out) {
    cMtx_copy(in, out);
    out[0][3] = out[1][3] = out[2][3] = 0.0f;
}

int joint_named(J3DModelData* data, const char* name) {
    JUTNameTab* names = data->getJointName();
    if (names == nullptr || name == nullptr) return -1;
    for (u16 i = 0; i < data->getJointNum(); ++i) {
        const char* n = names->getName(i);
        if (n != nullptr && std::strcmp(n, name) == 0) return i;
    }
    return -1;
}

void walk_him(J3DJoint* joint, const Mtx parentWorld, int parent, Mtx* world, int& n) {
    for (; joint != nullptr; joint = joint->getYounger()) {
        const int j = joint->getJntNo();
        if (j >= kMaxJoints) continue;
        const J3DTransformInfo& info = joint->getTransformInfo();
        Mtx local;
        J3DGetTranslateRotateMtx(info, local);
        MTXConcat(parentWorld, local, world[j]);
        rotation_only(local, s_restLocal[j]);
        s_restAngle[j] = info.mRotation;
        s_offset[j] = info.mTranslate;
        s_parent[j] = parent;
        s_order[n++] = j;
        walk_him(joint->getChild(), world[j], j, world, n);
    }
}

void walk_world(J3DJoint* joint, const Mtx parentWorld, Mtx* world) {
    for (; joint != nullptr; joint = joint->getYounger()) {
        const int j = joint->getJntNo();
        if (j >= kMaxJoints) continue;
        Mtx local;
        J3DGetTranslateRotateMtx(joint->getTransformInfo(), local);
        MTXConcat(parentWorld, local, world[j]);
        walk_world(joint->getChild(), world[j], world);
    }
}

void learn(J3DModelData* data) {
    if (s_learned) return;
    s_count = data->getJointNum();
    Mtx identity;
    MTXIdentity(identity);
    Mtx world[kMaxJoints];
    int n = 0;
    walk_him(data->getJointNodePointer(0), identity, -1, world, n);
    for (int j = 0; j < s_count; ++j) rotation_only(world[j], s_restWorld[j]);
    s_rootHeight = world[0][1][3] > 1.0f ? world[0][1][3] : 1.0f;
    for (int f = 0; f < 4; ++f) s_fingerJoint[f] = joint_named(data, kFingers[f].name);
    const int head = joint_named(data, "head");
    for (int k = 0; k < s_count; ++k) {
        const int j = s_order[k];
        s_inHead[j] = j == head || (s_parent[j] >= 0 && s_inHead[s_parent[j]]);
    }
    s_learned = true;
}

struct LinkRest {
    J3DModelData* data = nullptr;
    u16 joints = 0;
    f32 rootY = 0.0f;
    Mtx rest[kMaxJoints];
    int follow[kMaxJoints];
    f32 scale = 1.0f;
};
LinkRest s_linkRests[4];
int s_linkRestNext = 0;

LinkRest* link_rest(J3DModelData* data, J3DModelData* his) {
    if (data == nullptr || data->getJointNum() == 0 || data->getJointNum() > kMaxJoints) return nullptr;
    const f32 rootY = data->getJointNodePointer(0)->getTransformInfo().mTranslate.y;
    for (LinkRest& r : s_linkRests) {
        if (r.data == data && r.joints == data->getJointNum() && r.rootY == rootY) return &r;
    }
    LinkRest& r = s_linkRests[s_linkRestNext];
    s_linkRestNext = (s_linkRestNext + 1) % 4;
    Mtx identity;
    MTXIdentity(identity);
    Mtx world[kMaxJoints];
    walk_world(data->getJointNodePointer(0), identity, world);
    for (int j = 0; j < data->getJointNum(); ++j) rotation_only(world[j], r.rest[j]);
    r.data = data;
    r.joints = data->getJointNum();
    r.rootY = rootY;
    r.scale = world[0][1][3] > 1.0f ? world[0][1][3] / s_rootHeight : 1.0f;
    JUTNameTab* names = his->getJointName();
    for (int j = 0; j < s_count; ++j) {
        const char* name = names != nullptr ? names->getName(static_cast<u16>(j)) : nullptr;
        r.follow[j] = name != nullptr ? joint_named(data, name) : -1;
    }
    return &r;
}

struct Rig {
    J3DModel* model = nullptr;
    mDoExt_btkAnm* eyes = nullptr;
    mDoExt_btpAnm* blink = nullptr;
    mDoExt_brkAnm* core = nullptr;
    int blinkWait = 120;
    f32 blinkFrame = 0.0f;
    f32 grip[2] = {0.0f, 0.0f};
    f32 scale = 1.0f;
    Mtx pose[kMaxJoints];
    Mtx rigid[kMaxJoints];
};
Rig s_rig;
bool s_rigFailed = false;

J3DAnmTextureSRTKey* s_eyeAnm = nullptr;
J3DAnmTexPattern* s_blinkAnm = nullptr;
J3DAnmTevRegKey* s_coreAnm = nullptr;
bool s_animsLoaded = false;

const Mtx* s_posing = nullptr;
int s_posingCount = 0;

int pose_callback(J3DJoint* joint, int phase) {
    if (phase != 0 || s_posing == nullptr) return 1;
    const u16 j = joint->getJntNo();
    J3DModel* model = j3dSys.getModel();
    if (j >= s_posingCount || model == nullptr) return 1;
    Mtx m;
    cMtx_copy(s_posing[j], m);
    model->setAnmMtx(j, m);
    cMtx_copy(m, J3DSys::mCurrentMtx);
    return 1;
}

bool rig_build() {
    J3DModelData* data = arc_load(kArc, kBody);
    if (data == nullptr) return false;
    if (data->getJointNum() > kMaxJoints) {
        arc_free_data(data);
        return false;
    }
    J3DModel* model = ganon_create_model(data, 0x80000, 0x11000284);
    if (model == nullptr) {
        arc_free_data(data);
        return false;
    }
    for (u16 j = 0; j < data->getJointNum(); ++j) data->getJointNodePointer(j)->setCallBack(pose_callback);
    learn(data);
    if (!s_animsLoaded) {
        s_animsLoaded = true;
        s_eyeAnm = static_cast<J3DAnmTextureSRTKey*>(arc_load_anm(kArc, "eye_default.btk"));
        s_blinkAnm = static_cast<J3DAnmTexPattern*>(arc_load_anm(kArc, "egnd_mepachi.btp"));
        s_coreAnm = static_cast<J3DAnmTevRegKey*>(arc_load_anm(kArc, "egnd_core_beat.brk"));
    }

    JKRHeap* heap = arc_heap_if_any();
    JKRHeap* previous = heap != nullptr ? heap->becomeCurrentHeap() : nullptr;
    if (s_eyeAnm != nullptr) {
        s_rig.eyes = JKR_NEW mDoExt_btkAnm();
        if (s_rig.eyes != nullptr && !s_rig.eyes->init(data, s_eyeAnm, 1, 0, 1.0f, 0, -1)) s_rig.eyes = nullptr;
    }
    if (s_blinkAnm != nullptr) {
        s_rig.blink = JKR_NEW mDoExt_btpAnm();
        if (s_rig.blink != nullptr && !s_rig.blink->init(data, s_blinkAnm, 1, 2, 1.0f, 0, -1)) s_rig.blink = nullptr;
    }
    if (s_coreAnm != nullptr) {
        s_rig.core = JKR_NEW mDoExt_brkAnm();
        if (s_rig.core != nullptr && !s_rig.core->init(data, s_coreAnm, 1, 2, 1.0f, 0, -1)) s_rig.core = nullptr;
    }
    if (previous != nullptr) previous->becomeCurrentHeap();
    s_rig.model = model;
    s_rig.blinkWait = 60 + std::rand() % 180;
    mods::log::info("ganondorf: he is ready ({} joints, eyes {} blink {} core {})", s_count,
        s_rig.eyes != nullptr, s_rig.blink != nullptr, s_rig.core != nullptr);
    return true;
}

int finger_of(int joint) {
    for (int f = 0; f < 4; ++f) {
        if (s_fingerJoint[f] == joint) return f;
    }
    return -1;
}

void retarget(J3DModel* link, const LinkRest& lr) {
    Mtx rot[kMaxJoints];
    Vec pos[kMaxJoints];
    const f32 s = s_rig.scale;
    for (int k = 0; k < s_count; ++k) {
        const int j = s_order[k];
        const int p = s_parent[j];
        const int l = lr.follow[j];
        const int finger = finger_of(j);
        if (l >= 0) {
            Mtx now, inv, delta;
            rotation_only(link->getAnmMtx(l), now);
            MTXInverse(lr.rest[l], inv);
            MTXConcat(now, inv, delta);
            MTXConcat(delta, s_restWorld[j], rot[j]);
        } else if (p >= 0 && finger >= 0) {
            const Finger& f = kFingers[finger];
            const f32 bend = s_rig.grip[f.right ? 1 : 0] * f.closeDegrees * (f.right ? -1.0f : 1.0f);
            const S16Vec& a = s_restAngle[j];
            Mtx local;
            J3DGetTranslateRotateMtx(a.x, static_cast<s16>(a.y + bend * (32768.0f / 180.0f)), a.z, 0.0f,
                0.0f, 0.0f, local);
            MTXConcat(rot[p], local, rot[j]);
        } else if (p >= 0) {
            MTXConcat(rot[p], s_restLocal[j], rot[j]);
        } else {
            rotation_only(link->getAnmMtx(0), rot[j]);
        }
        if (p < 0) {
            MtxP centre = link->getAnmMtx(l >= 0 ? l : 0);
            pos[j].x = centre[0][3];
            pos[j].y = centre[1][3];
            pos[j].z = centre[2][3];
        } else {
            Vec along;
            MTXMultVec(rot[p], &s_offset[j], &along);

            const f32 bone = s_inHead[p] ? s * kHeadScale : s;
            pos[j].x = pos[p].x + along.x * bone;
            pos[j].y = pos[p].y + along.y * bone;
            pos[j].z = pos[p].z + along.z * bone;
        }
        const f32 drawn = s_inHead[j] ? s * kHeadScale : s;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                s_rig.pose[j][r][c] = rot[j][r][c] * drawn;
                s_rig.rigid[j][r][c] = rot[j][r][c];
            }
        }
        s_rig.pose[j][0][3] = s_rig.rigid[j][0][3] = pos[j].x;
        s_rig.pose[j][1][3] = s_rig.rigid[j][1][3] = pos[j].y;
        s_rig.pose[j][2][3] = s_rig.rigid[j][2][3] = pos[j].z;
    }
}

int his_joint_for(const LinkRest& lr, int l) {
    for (int j = 0; j < s_count; ++j) {
        if (lr.follow[j] == l) return j;
    }
    return -1;
}

void blink() {
    if (s_rig.blink == nullptr) return;
    if (s_rig.blinkFrame > 0.0f || --s_rig.blinkWait <= 0) {
        s_rig.blinkFrame += 1.0f;
        if (s_rig.blinkFrame > s_rig.blink->getEndFrame()) {
            s_rig.blinkFrame = 0.0f;
            s_rig.blinkWait = 90 + std::rand() % 210;
        }
    }
}

void approach(f32& value, f32 target) {
    const f32 step = 0.2f;
    if (value < target) {
        value = value + step > target ? target : value + step;
    } else {
        value = value - step < target ? target : value - step;
    }
}

void rig_draw(J3DModel* link, bool gripL, bool gripR, dKy_tevstr_c* tev) {
    LinkRest* lr = link_rest(link->getModelData(), s_rig.model->getModelData());
    if (lr == nullptr) return;
    s_rig.scale = lr->scale;
    approach(s_rig.grip[0], gripL ? 1.0f : 0.0f);
    approach(s_rig.grip[1], gripR ? 1.0f : 0.0f);
    retarget(link, *lr);

    J3DModelData* data = s_rig.model->getModelData();
    if (s_rig.eyes != nullptr) s_rig.eyes->entry(data, 0.0f);
    if (s_rig.core != nullptr) {
        s_rig.core->entry(data);
        s_rig.core->play();
    }
    blink();
    if (s_rig.blink != nullptr) s_rig.blink->entry(data, static_cast<s16>(s_rig.blinkFrame));

    s_rig.model->setBaseTRMtx(link->getBaseTRMtx());
    s_posing = s_rig.pose;
    s_posingCount = s_count;
    s_rig.model->calc();
    s_posing = nullptr;
    g_env_light.setLightTevColorType_MAJI(s_rig.model, tev);
    mDoExt_modelEntryDL(s_rig.model);
    daMirror_c::entry(s_rig.model);
}

bool gripping(daAlink_c* alink, J3DShape* shown) {
    if (shown == nullptr || alink->mpLinkHandModel == nullptr) return false;
    J3DModelData* hands = alink->mpLinkHandModel->getModelData();
    for (u16 i = 0; i < hands->getMaterialNum(); ++i) {
        J3DMaterial* mat = hands->getMaterialNodePointer(i);
        if (mat != nullptr && mat->getShape() == shown) return true;
    }
    return false;
}

const f32 kSwordScale = 0.555f;
const f32 kSwordGrip = 38.3f;
const f32 kSheathMouth = 60.9f;

const f32 kSheathAtGuard = 11.0f;

J3DModelData* s_swordData = nullptr;
J3DModelData* s_sheathData = nullptr;
J3DModel* s_sword = nullptr;
J3DModel* s_sheath = nullptr;
bool s_swordFailed = false;

J3DModelData* rehung(const char* file, f32 along, bool sheath) {
    J3DModelData* data = arc_load(kArc, file);
    if (data == nullptr || data->getJointNum() < 2) return data;
    J3DTransformInfo& root = data->getJointNodePointer(0)->getTransformInfo();
    J3DTransformInfo& mesh = data->getJointNodePointer(1)->getTransformInfo();
    if (!sheath) {

        root.mScale.x = root.mScale.y = root.mScale.z = kSwordScale;
        root.mRotation.x = 0;
        root.mRotation.y = static_cast<s16>(0xC000);
        root.mRotation.z = static_cast<s16>(0xC000);
        root.mTranslate.x = root.mTranslate.y = root.mTranslate.z = 0.0f;
        mesh.mRotation.x = mesh.mRotation.y = mesh.mRotation.z = 0;
        mesh.mTranslate.x = 0.0f;
        mesh.mTranslate.y = -along;
        mesh.mTranslate.z = 0.0f;
    } else {

        root.mScale.x = root.mScale.y = root.mScale.z = 1.0f;
        root.mRotation.x = 0;
        root.mRotation.y = cM_deg2s(33.1f);
        root.mRotation.z = 0;
        root.mTranslate.x = -18.5f;
        root.mTranslate.y = 0.14f;
        root.mTranslate.z = 12.2f;
        mesh.mScale.x = mesh.mScale.y = mesh.mScale.z = kSwordScale;
        mesh.mRotation.x = 0;
        mesh.mRotation.y = static_cast<s16>(0xC000);
        mesh.mRotation.z = static_cast<s16>(0xC000);
        mesh.mTranslate.x = kSheathAtGuard - along * kSwordScale;
        mesh.mTranslate.y = 0.0f;
        mesh.mTranslate.z = 0.0f;
    }

    for (u16 i = 0; i < data->getMaterialNum(); ++i) {
        J3DMaterial* mat = data->getMaterialNodePointer(i);
        J3DAlphaComp* comp =
            mat != nullptr && mat->getPEBlock() != nullptr ? mat->getPEBlock()->getAlphaComp() : nullptr;
        if (comp == nullptr) continue;
        const J3DAlphaCompInfo always = {GX_ALWAYS, 0, GX_AOP_AND, GX_ALWAYS, 0};
        comp->setAlphaCompInfo(always);
    }
    return data;
}

bool build_sword() {
    if (s_sword != nullptr && s_sheath != nullptr) return true;
    if (s_swordFailed || !arc_ready()) return false;
    if (s_swordData == nullptr) s_swordData = rehung("egnd_sword.bmd", kSwordGrip, false);
    if (s_sheathData == nullptr) s_sheathData = rehung("egnd_sheath.bmd", kSheathMouth, true);
    if (s_swordData == nullptr || s_sheathData == nullptr) {
        s_swordFailed = true;
        mods::log::warn("ganondorf: his sword could not be loaded");
        return false;
    }
    if (s_sword == nullptr) s_sword = ganon_create_model(s_swordData, 0x80000, 0x11000284);
    if (s_sheath == nullptr) s_sheath = ganon_create_model(s_sheathData, 0x80000, 0x11000284);
    s_swordFailed = s_sword == nullptr || s_sheath == nullptr;
    return !s_swordFailed;
}

dKy_tevstr_c s_swordTev;
int s_swordTevRoom = -1000;

void draw_in_place(J3DModel* ours, J3DModel* links, daAlink_c* alink) {
    ours->setBaseTRMtx(links->getBaseTRMtx());
    ours->calc();
    dKy_tevstr_c* tev = &alink->tevStr;
    if (ours == s_sword) {
        const int room = fopAcM_GetRoomNo(alink);
        if (room != s_swordTevRoom) {
            dKy_tevstr_init(&s_swordTev, static_cast<s8>(room), 0xFF);
            s_swordTevRoom = room;
        }
        g_env_light.settingTevStruct(5, &alink->current.pos, &s_swordTev);
        tev = &s_swordTev;
    }
    g_env_light.setLightTevColorType_MAJI(ours, tev);
    mDoExt_modelEntryDL(ours);
}

int sword_kind(daAlink_c* alink) {
    J3DModel* sword = alink->mSwordModel;
    if (sword == nullptr) return -1;
    if (sword == alink->mWoodSwordModel) return kSwordWood;
    if (sword == alink->mpSwAModel) return kSwordOrdon;
    if (sword == alink->mpSwMModel) return kSwordMaster;
    return -1;
}

bool sword_wanted(daAlink_c* alink) {
    const GanonOptions& o = ganon_options();
    const int kind = sword_kind(alink);
    return o.sword && kind >= 0 && o.swordFor[kind];
}

int outfit_of() {
    switch (dComIfGs_getSelectEquipClothes()) {
    case dItemNo_WEAR_CASUAL_e: return kOutfitOrdon;
    case dItemNo_WEAR_ZORA_e: return kOutfitZora;
    case dItemNo_ARMOR_e: return kOutfitMagic;
    default: return kOutfitHero;
    }
}

bool body_wanted(daAlink_c* alink) {
    if (alink == nullptr || alink->mClothesChangeWaitTimer != 0) return false;
    if (alink->checkWolf() || alink->mpLinkModel == nullptr) return false;
    const GanonOptions& o = ganon_options();
    return o.body && (o.allOutfits || o.outfit[outfit_of()]);
}

}

bool ganon_is_him(daAlink_c* alink) {
    const GanonOptions& o = ganon_options();
    return !alink->checkWolf() && o.body && (o.allOutfits || o.outfit[outfit_of()]);
}

namespace {

bool standing_in(daAlink_c* alink) {
    return s_rig.model != nullptr && body_wanted(alink);
}

HookAction on_model_draw_pre(ModContext*, void* args, void*, void*) {
    daAlink_c* alink = mods::arg<daAlink_c*>(args, 0);
    J3DModel* model = mods::arg<J3DModel*>(args, 1);
    const int noDraw = mods::arg<int>(args, 2);
    if (alink != daAlink_getAlinkActorClass() || model == nullptr) return HOOK_CONTINUE;

    if ((model == alink->mSwordModel || model == alink->mSheathModel) && sword_wanted(alink) &&
        build_sword()) {
        if (noDraw == 0) draw_in_place(model == alink->mSwordModel ? s_sword : s_sheath, model, alink);
        return HOOK_SKIP_ORIGINAL;
    }
    if (!standing_in(alink)) return HOOK_CONTINUE;
    const bool body = model == alink->mpLinkModel || model == alink->mpLinkHandModel ||
                      model == alink->mpLinkHatModel || model == alink->mpLinkFaceModel ||
                      model == alink->mpLinkBootModels[0] || model == alink->mpLinkBootModels[1];
    return body ? HOOK_SKIP_ORIGINAL : HOOK_CONTINUE;
}

struct HeldSwap {
    int link;
    Mtx saved;
};
const int kHeldMax = 8;
HeldSwap s_held[kHeldMax];
int s_heldCount = 0;

void hold_begin(J3DModel* link, const LinkRest& lr, const int* joints, int count, int pod) {
    s_heldCount = 0;
    for (int n = 0; n < count; ++n) {
        const int l = joints[n];
        if (l < 0 || l >= lr.joints) continue;
        bool already = false;
        for (int i = 0; i < s_heldCount; ++i) already = already || s_held[i].link == l;
        const int j = his_joint_for(lr, l);
        if (already || j < 0 || s_heldCount >= kHeldMax) continue;
        HeldSwap& h = s_held[s_heldCount++];
        h.link = l;
        cMtx_copy(link->getAnmMtx(l), h.saved);
        link->setAnmMtx(l, s_rig.rigid[j]);
    }

    const int spine = pod >= 0 && pod < lr.joints ? his_joint_for(lr, 2) : -1;
    if (spine >= 0 && s_heldCount < kHeldMax) {
        const J3DTransformInfo& info =
            link->getModelData()->getJointNodePointer(static_cast<u16>(pod))->getTransformInfo();
        Mtx local, at;
        J3DGetTranslateRotateMtx(info.mRotation.x, info.mRotation.y, info.mRotation.z, info.mTranslate.x,
            info.mTranslate.y * kBackDepth, info.mTranslate.z, local);
        MTXConcat(s_rig.rigid[spine], local, at);
        HeldSwap& h = s_held[s_heldCount++];
        h.link = pod;
        cMtx_copy(link->getAnmMtx(pod), h.saved);
        link->setAnmMtx(pod, at);
    }
}

HookAction on_item_matrix_pre(ModContext*, void* args, void*, void*) {
    s_heldCount = 0;
    daAlink_c* alink = mods::arg<daAlink_c*>(args, 0);
    if (alink != daAlink_getAlinkActorClass() || !standing_in(alink)) return HOOK_CONTINUE;
    J3DModel* link = alink->mpLinkModel;
    LinkRest* lr = link_rest(link->getModelData(), s_rig.model->getModelData());
    if (lr == nullptr) return HOOK_CONTINUE;
    s_rig.scale = lr->scale;
    retarget(link, *lr);
    const int joints[3] = {alink->mLeftItemJntNo, alink->mRightItemJntNo, 4};
    hold_begin(link, *lr, joints, 3, alink->field_0x30b6);
    return HOOK_CONTINUE;
}

void on_item_matrix_post(ModContext*, void* args, void*, void*) {
    daAlink_c* alink = mods::arg<daAlink_c*>(args, 0);
    J3DModel* link = alink != nullptr ? alink->mpLinkModel : nullptr;
    if (link != nullptr) {
        for (int i = 0; i < s_heldCount; ++i) link->setAnmMtx(s_held[i].link, s_held[i].saved);
    }
    s_heldCount = 0;
}

struct ShadowSwap {
    bool on = false;
    J3DModel* body = nullptr;
    J3DModel* face = nullptr;
    J3DModel* hat = nullptr;
    J3DModel* hand = nullptr;
};
ShadowSwap s_shadow;

HookAction on_shadow_pre(ModContext*, void* args, void*, void*) {
    s_shadow.on = false;
    daAlink_c* alink = mods::arg<daAlink_c*>(args, 0);
    if (alink != daAlink_getAlinkActorClass() || !standing_in(alink)) return HOOK_CONTINUE;
    s_shadow.on = true;
    s_shadow.body = alink->mpLinkModel;
    s_shadow.face = alink->mpLinkFaceModel;
    s_shadow.hat = alink->mpLinkHatModel;
    s_shadow.hand = alink->mpLinkHandModel;
    alink->mpLinkModel = s_rig.model;
    alink->mpLinkFaceModel = nullptr;
    alink->mpLinkHatModel = nullptr;
    alink->mpLinkHandModel = nullptr;
    return HOOK_CONTINUE;
}

void on_shadow_post(ModContext*, void* args, void*, void*) {
    if (!s_shadow.on) return;
    s_shadow.on = false;
    daAlink_c* alink = mods::arg<daAlink_c*>(args, 0);
    if (alink == nullptr) return;
    alink->mpLinkModel = s_shadow.body;
    alink->mpLinkFaceModel = s_shadow.face;
    alink->mpLinkHatModel = s_shadow.hat;
    alink->mpLinkHandModel = s_shadow.hand;
}

void on_draw_post(ModContext*, void* args, void*, void*) {
    daAlink_c* alink = mods::arg<daAlink_c*>(args, 0);
    if (alink == daAlink_getAlinkActorClass()) ganon_frame(alink);
}

}

void ganon_frame(daAlink_c* alink) {
    const GanonOptions& o = ganon_options();

    if (s_arc == kArcMounting || (s_arc == kArcNone && (o.body || o.sword))) arc_ready();
    if (!body_wanted(alink) || !arc_ready()) return;
    if (s_rig.model == nullptr) {
        if (s_rigFailed) return;
        if (!rig_build()) {
            s_rigFailed = true;
            mods::log::warn("ganondorf: he could not be loaded");
            return;
        }
    }
    const bool gripL = gripping(alink, alink->field_0x06d0);
    const bool gripR = gripping(alink, alink->field_0x06d4);
    rig_draw(alink->mpLinkModel, gripL, gripR, &alink->tevStr);
}

void ganon_init() {
    const int results[] = {
        static_cast<int>(mods::hook::add_pre<GanonModelDrawHook>(on_model_draw_pre)),
        static_cast<int>(mods::hook::add_pre<GanonItemMatrixHook>(on_item_matrix_pre)),
        static_cast<int>(mods::hook::add_post<GanonItemMatrixHook>(on_item_matrix_post)),
        static_cast<int>(mods::hook::add_pre<GanonShadowHook>(on_shadow_pre)),
        static_cast<int>(mods::hook::add_post<GanonShadowHook>(on_shadow_post)),
        static_cast<int>(mods::hook::add_post<GanonAlinkDrawHook>(on_draw_post)),
    };
    int failed = 0;
    for (int r : results) failed += r != 0 ? 1 : 0;
    mods::log::info("ganondorf: {} of 6 hooks in", 6 - failed);
    voice_init();
}
