{
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

  outputs = { self, nixpkgs, ... }:
    let
      system = "aarch64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      packages.${system} =
        let
          # button service
          daemonPython = pkgs.python3.withPackages (ps: with ps; [
            pyserial
            lgpio
          ]);

          # pdf2gcode
          toolPython = pkgs.python3.withPackages (ps: with ps; [
            numpy
            pillow
            scikit-image
          ]);
        in rec {
          blotd = pkgs.writeShellApplication {
            name = "blotd";
            runtimeInputs = [ daemonPython ];
            text = ''
              exec python3 ${self}/buttonService.py "$@"
            '';
          };

          blot-web = pkgs.writeShellApplication {
            name = "blot-web";
            runtimeInputs = [ pkgs.bun toolPython pkgs.poppler-utils ];
            text = ''
              export PDF2GCODE=${self}/tools/pdf2gcode.py
              export BLOT_REPO=${self}
              exec bun run ${self}/webserver.ts "$@"
            '';
          };

          default = blotd;
        };
    };
}
