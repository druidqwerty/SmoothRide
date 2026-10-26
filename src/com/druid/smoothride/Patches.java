package com.druid.smoothride;

import me.zed_0xff.zombie_buddy.Patch;
import zombie.vehicles.BaseVehicle;
import zombie.vehicles.VehicleInterpolation;

/** ZombieBuddy advice. Bodies only forward to {@link Engine}; all logic and error handling lives there. */
public class Patches {

    /** Every snapshot of a remote vehicle (physics packets and server position updates) passes here. */
    @Patch(className = "zombie.vehicles.VehicleInterpolation", methodName = "interpolationDataAdd")
    public static class InterpAdd {
        @Patch.OnEnter(skipOn = true)
        public static boolean enter(@Patch.This Object self, @Patch.Argument(0) BaseVehicle vehicle, @Patch.Argument(1) Object data) {
            return Engine.onAdd(self, vehicle, data);
        }
    }

    /** Pose of a remote vehicle for this frame. Only the 3-argument overload VehicleManager calls. */
    @Patch(className = "zombie.vehicles.VehicleInterpolation", methodName = "interpolationDataGet", strictMatch = true)
    public static class InterpGet {
        @Patch.OnEnter(skipOn = true)
        public static boolean enter(@Patch.This Object self, @Patch.Argument(0) Object buf1, @Patch.Argument(1) Object buf2,
                                    @Patch.Argument(2) VehicleInterpolation towedBy) {
            return Engine.onGetEnter(self, buf1, buf2, towedBy);
        }

        @Patch.OnExit
        public static void exit(@Patch.Return(readOnly = false) boolean ret) {
            if (Engine.consumeHandled()) {
                ret = Engine.pendingResult();
            }
        }
    }

    /** reset(), clearAll() and vehicle removal all end up here. */
    @Patch(className = "zombie.vehicles.VehicleInterpolation", methodName = "clear")
    public static class InterpClear {
        @Patch.OnEnter
        public static void enter(@Patch.This Object self) {
            Engine.onClear(self);
        }
    }

    /** Driver side: shorten the physics send period while the car moves. */
    @Patch(className = "zombie.vehicles.BaseVehicle", methodName = "updatePhysicsNetwork")
    public static class SendRate {
        @Patch.OnExit
        public static void exit(@Patch.This BaseVehicle self) {
            Engine.afterPhysicsSend(self);
        }
    }

    /** Marks snapshots that come from the server's own VehicleUpdate packet. */
    @Patch(className = "zombie.network.packets.vehicle.VehicleUpdatePacket", methodName = "parse")
    public static class ServerUpdateMark {
        @Patch.OnEnter
        public static void enter() {
            Engine.serverUpdateEnter();
        }

        @Patch.OnExit(onThrowable = Throwable.class)
        public static void exit() {
            Engine.serverUpdateExit();
        }
    }

    /** Once per frame on a multiplayer client: enter before remote vehicles are positioned, exit after. */
    @Patch(className = "zombie.vehicles.VehicleManager", methodName = "clientUpdate")
    public static class Frame {
        @Patch.OnEnter
        public static void enter() {
            Engine.frame();
        }

        @Patch.OnExit(onThrowable = Throwable.class)
        public static void exit() {
            Engine.frameEnd();
        }
    }
}
