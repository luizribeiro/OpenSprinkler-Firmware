{ pkgs ? import <nixpkgs> { } }:

pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    # dev tools
    ccls
    clang-tools
    gdb

    # dependencies
    cmake
    gcc9
    libgpiod
    mosquitto
  ];
}
