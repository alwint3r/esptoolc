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
            "src/os_hal.c",
        },
        .flags = &[_][]const u8{
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-std=c11",
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

    const exe = b.addExecutable(.{
        .name = "esp-download-mode",
        .root_module = module,
    });

    b.installArtifact(serial_port_lib);
    b.installArtifact(exe);

    const cmd = b.addRunArtifact(exe);
    cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run this application");
    run_step.dependOn(&cmd.step);
}
