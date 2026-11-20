// Drone registry: all DroneSpec definitions + Catalog().
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_REGISTRY_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_REGISTRY_H_
#include "drone_sine.h"
#include "drone_square.h"
#include "drone_triangle.h"
#include "drone_harms_way.h"
#include "drone_thx.h"
#include "drone_mt_zion.h"
#include "drone_mt_lion.h"
#include "drone_hecker.h"
#include "drone_fieldsteel.h"
#include "drone_apparatus.h"
#include "drone_twin_pks.h"
#include "drone_supersaw.h"
#include "drone_starlids.h"
#include "drone_sachiko.h"
#include "drone_coil.h"

namespace sc {
// 160 KiB arena alongside the shared KitTables. Checked at compile time.
static_assert(sizeof(drones::HeckerState) < 128 * 1024, "HeckerState too large");
static_assert(sizeof(drones::FsState) < 128 * 1024, "Fieldsteel FsState too large");
static_assert(sizeof(drones::ApState) < 128 * 1024, "Apparatus ApState too large");
static_assert(sizeof(drones::TpState) < 128 * 1024, "Twin Pks TpState too large");
static_assert(sizeof(drones::SsState) < 128 * 1024, "Supersaw SsState too large");
static_assert(sizeof(drones::StState) < 128 * 1024, "Starlids StState too large");
static_assert(sizeof(drones::SkState) < 128 * 1024, "Sachiko SkState too large");
static_assert(sizeof(drones::ClState) < 128 * 1024, "Coil ClState too large");


inline const DroneSpec kDroneHarmsWay = {"harms way", drones::HarmsWay_Init, drones::HarmsWay_Process};
inline const DroneSpec kDroneThx = {"Thx", drones::Thx_Init, drones::Thx_Process};
inline const DroneSpec kDroneMtZion = {"Mt. Zion", drones::Mz_Init, drones::Mz_Process};
inline const DroneSpec kDroneMtLion = {"Mt. Lion", drones::Ml_Init, drones::Ml_Process};
inline const DroneSpec kDroneHecker = {"Hecker", drones::Hk_Init, drones::Hk_Process};
inline const DroneSpec kDroneFieldsteel = {"Fieldsteel", drones::Fs_Init, drones::Fs_Process};
inline const DroneSpec kDroneApparatus = {"Apparatus", drones::Ap_Init, drones::Ap_Process};
inline const DroneSpec kDroneTwinPks = {"Twin Pks", drones::Tp_Init, drones::Tp_Process};
inline const DroneSpec kDroneSupersaw = {"Supersaw", drones::Ss_Init, drones::Ss_Process};
inline const DroneSpec kDroneStarlids = {"Starlids", drones::St_Init, drones::St_Process};
inline const DroneSpec kDroneSachiko = {"Sachiko", drones::Sk_Init, drones::Sk_Process};
inline const DroneSpec kDroneCoil = {"Coil", drones::Cl_Init, drones::Cl_Process};
inline const DroneSpec kDroneSine = {"Sine", drones::Sine_Init, drones::Sine_Process};
inline const DroneSpec kDroneSquare = {"Square", drones::Square_Init, drones::Square_Process};
inline const DroneSpec kDroneTriangle = {"Triangle", drones::Triangle_Init, drones::Triangle_Process};

inline const DroneSpec* Catalog(int id) {
  static const DroneSpec* const kCatalog[kCatalogCount] = {
      /* 0 Apparatus .. 13 */ &kDroneApparatus, nullptr,
      nullptr,  // id 2 Coil DEFERRED (different timbre; see drone_coil.h)
      nullptr, nullptr,
      nullptr, nullptr, &kDroneFieldsteel,
      nullptr, nullptr, &kDroneHecker, nullptr, nullptr, nullptr,
      /* 14 */ &kDroneMtLion,
      /* 15 */ &kDroneMtZion,
      /* 16 .. 21 */ nullptr, nullptr, nullptr, nullptr,
      &kDroneSachiko, nullptr,
      /* 22 */ &kDroneSine, /* 23 */ &kDroneSquare,
      /* 24 */ &kDroneStarlids,
      /* 25 */ &kDroneSupersaw,
      /* 26 */ nullptr,
      /* 27 */ &kDroneThx,
      /* 28 */ nullptr,
      /* 29 */ &kDroneTriangle,
      /* 30 .. 35 */ &drones::kDroneTwinPks, nullptr, nullptr, nullptr, nullptr,
      nullptr,
      /* 36 */ &kDroneHarmsWay,
  };
  if (id < 0 || id >= kCatalogCount) return nullptr;
  return kCatalog[id];
}

}  // namespace sc
#endif
