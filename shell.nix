{ pkgs ? import <nixpkgs> { } }:

pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    gcc9
    libgpiod
    mosquitto
  ];
}
