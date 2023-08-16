{
  description = "Basic rpi pico development shell";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/master";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system: 
    let
      pico-sdk140 = with pkgs; (pico-sdk.overrideAttrs (o:
        rec {
        pname = "pico-sdk";
        version = "1.4.0";
        src = fetchFromGitHub {
          fetchSubmodules = true;
          owner = "raspberrypi";
          repo = pname;
          rev = version;
          sha256 = "sha256:1wihm752wm3mnnrqnr7vjvrlzrhpd418jgf9i5bfajri45qrl6vs";
        };
        }));
      pkgs = nixpkgs.legacyPackages.${system};
    in {
        devShell = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [
            pico-sdk140 
            cmake
            clang-tools
            gcc-arm-embedded
            ];
          shellHook = ''
            export PICO_SDK_PATH="${pico-sdk140}/lib/pico-sdk"
            '';
          };
      });
}