using System.Reflection;
using System.Runtime.InteropServices;

static class Native {
    const string Library = "gravity";
    [StructLayout(LayoutKind.Sequential)] public struct Vec3 { public long X, Y, Z; }
    [StructLayout(LayoutKind.Sequential)] public struct Quat { public long X, Y, Z, W; }
    [StructLayout(LayoutKind.Sequential)] public struct Transform { public Vec3 Position; public Quat Orientation; }
    [StructLayout(LayoutKind.Sequential)] public struct AssetStoreDesc { public uint StructSize, Reserved; public nint Assets; public uint AssetCount, Reserved1; }
    [StructLayout(LayoutKind.Sequential)] public struct WorldDesc {
        public uint StructSize, Reserved, BodyCapacity, ColliderCapacity, CommandCapacity, ContactCapacity;
        public Vec3 Gravity; public long LinearDamping, AngularDamping, MaxLinearSpeed, MaxAngularSpeed;
        public uint Substeps, TickHz; public nint Assets;
    }
    [StructLayout(LayoutKind.Sequential)] public struct BodyDesc {
        public uint StructSize, Reserved, BodyType, DofLocks; public Transform Transform; public long InverseMass;
        public long Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    }
    [StructLayout(LayoutKind.Sequential)] public unsafe struct Hash128 { public fixed byte Bytes[16]; }
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_abi_version();
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_asset_store_memory_required(ref AssetStoreDesc desc, out ulong size, out uint alignment);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_asset_store_init(nint memory, ulong size, ref AssetStoreDesc desc, out nint store);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_asset_store_deinit(nint store);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_memory_required(ref WorldDesc desc, out ulong size, out uint alignment);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_init(nint memory, ulong size, ref WorldDesc desc, out nint world);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_deinit(nint world);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_create_body(nint world, ref BodyDesc desc, out ulong id);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_hash(nint world, out Hash128 hash);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_snapshot_size(nint world, out ulong size);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_snapshot_save(nint world, nint output, ulong capacity, out ulong required);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl)] public static extern uint gravity_v1_world_snapshot_load(nint world, nint input, ulong length);
}

unsafe class Program {
    static void Ok(uint result, string where) { if (result != 0) throw new Exception($"{where}: {result}"); }
    static byte[] Bytes(Native.Hash128 hash) { var result = new byte[16]; for (int i = 0; i < result.Length; ++i) result[i] = hash.Bytes[i]; return result; }
    static int Main(string[] args) {
        if (args.Length != 1) return 2;
        NativeLibrary.SetDllImportResolver(typeof(Native).Assembly, (name, _, _) => name == "gravity" ? NativeLibrary.Load(Path.GetFullPath(args[0])) : nint.Zero);
        if (Native.gravity_v1_abi_version() != 1) return 3;
        var assetDesc = new Native.AssetStoreDesc { StructSize = (uint)Marshal.SizeOf<Native.AssetStoreDesc>() };
        Ok(Native.gravity_v1_asset_store_memory_required(ref assetDesc, out var assetSize, out _), "asset memory");
        var assetMemory = Marshal.AllocHGlobal((nint)assetSize);
        Ok(Native.gravity_v1_asset_store_init(assetMemory, assetSize, ref assetDesc, out var assets), "asset init");
        long one = 1L << 32;
        var worldDesc = new Native.WorldDesc { StructSize=(uint)Marshal.SizeOf<Native.WorldDesc>(), BodyCapacity=4, ColliderCapacity=4, CommandCapacity=4, ContactCapacity=4, MaxLinearSpeed=long.MaxValue, MaxAngularSpeed=long.MaxValue, Substeps=2, TickHz=60, Assets=assets };
        Ok(Native.gravity_v1_world_memory_required(ref worldDesc, out var worldSize, out _), "world memory");
        var worldMemory = Marshal.AllocHGlobal((nint)worldSize);
        Ok(Native.gravity_v1_world_init(worldMemory, worldSize, ref worldDesc, out var world), "world init");
        var bodyDesc = new Native.BodyDesc { StructSize=(uint)Marshal.SizeOf<Native.BodyDesc>(), BodyType=1, Transform=new Native.Transform { Orientation=new Native.Quat { W=one } }, InverseMass=one, Ixx=one, Iyy=one, Izz=one };
        Ok(Native.gravity_v1_world_create_body(world, ref bodyDesc, out _), "body create");
        Ok(Native.gravity_v1_world_hash(world, out var hashA), "hash A");
        Ok(Native.gravity_v1_world_snapshot_size(world, out var snapshotSize), "snapshot size");
        var snapshot = Marshal.AllocHGlobal((nint)snapshotSize);
        Ok(Native.gravity_v1_world_snapshot_save(world, snapshot, snapshotSize, out _), "snapshot save");
        Ok(Native.gravity_v1_world_snapshot_load(world, snapshot, snapshotSize), "snapshot load");
        Ok(Native.gravity_v1_world_hash(world, out var hashB), "hash B");
        if (!Bytes(hashA).SequenceEqual(Bytes(hashB))) return 4;
        var hex = Convert.ToHexString(Bytes(hashA)).ToLowerInvariant();
        if (hex != "4336297d3f06a9c557e75aea2a839853") return 5;
        Console.WriteLine(hex);
        Marshal.FreeHGlobal(snapshot); Ok(Native.gravity_v1_world_deinit(world), "world deinit"); Marshal.FreeHGlobal(worldMemory);
        Ok(Native.gravity_v1_asset_store_deinit(assets), "asset deinit"); Marshal.FreeHGlobal(assetMemory);
        return 0;
    }
}
