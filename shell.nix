{ pkgs ? import <nixpkgs> { } }:

pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    # dev tools
    ccls
    clang-tools

    # dependencies
    cmake
    gcc9
    libgpiod
    mosquitto
  ];
}
