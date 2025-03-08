const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const serial_port_module = b.addModule("esp-serial-port", .{
        .link_libc = true,
        .optimize = optimize,
        .target = target,
    });

    serial_port_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "src/esp_serial_port.c",
            "src/commands/esp_command.c",
            "src/commands//esp_command_sync.c",
            "src/esp_chip.c",
            "src/osal.c",
            "src/slip_reader.c",
            "src/slip_writer.c",
        },
        .flags = &[_][]const u8{
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-std=c11",
            "-Isrc/",
        },
    });

    const serial_port_lib = b.addStaticLibrary(.{
        .name = "esp-serial-port",
        .root_module = serial_port_module,
    });

    const module = b.addModule("main", .{
        .link_libc = true,
        .optimize = optimize,
        .target = target,
    });

    module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "src/main.c",
        },
        .flags = &[_][]const u8{
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-std=c11",
        },
    });

    module.linkLibrary(serial_port_lib);

    const unity_module = b.addModule("unity", .{
        .link_libc = true,
        .optimize = optimize,
        .target = target,
    });

    unity_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "unity/src/unity.c",
        },
        .flags = &[_][]const u8{
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-std=c11",
        },
    });

    const unity_library = b.addStaticLibrary(.{
        .name = "unity",
        .root_module = unity_module,
    });

    const test_module = b.addModule("test", .{
        .link_libc = true,
        .optimize = optimize,
        .target = target,
    });

    test_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "test/test.c",
        },
        .flags = &[_][]const u8{
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-std=c11",
            "-Isrc/",
            "-Iunity/src",
        },
    });

    test_module.linkLibrary(serial_port_lib);
    test_module.linkLibrary(unity_library);

    const test_exe = b.addExecutable(.{
        .name = "test-esp-download-mode",
        .root_module = test_module,
    });

    const exe = b.addExecutable(.{
        .name = "esp-download-mode",
        .root_module = module,
    });

    b.installArtifact(serial_port_lib);
    b.installArtifact(exe);
    b.installArtifact(unity_library);
    b.installArtifact(test_exe);

    const cmd = b.addRunArtifact(exe);
    cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run this application");
    run_step.dependOn(&cmd.step);

    const test_cmd = b.addRunArtifact(test_exe);
    test_cmd.step.dependOn(b.getInstallStep());

    const test_run_step = b.step("test", "Run the tests");
    test_run_step.dependOn(&test_cmd.step);
}
