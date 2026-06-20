{
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

  outputs = { self, nixpkgs, ... }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in {
      packages = forAllSystems (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};

          # Runtime deps of buttonService.py (the GPIO/serial controller daemon).
          daemonPython = pkgs.python3.withPackages (ps: with ps; [
            pyserial
            gpiozero
            lgpio
          ]);

          # Runtime deps of tools/pdf2gcode.py (invoked by the web server).
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
              export GPIOZERO_PIN_FACTORY=lgpio
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
        });

      devShells = forAllSystems (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          python = pkgs.python3.withPackages (ps: with ps; [
            pyserial
            prompt-toolkit
            numpy
            pillow
            scikit-image
          ]);
        in {
          default = pkgs.mkShell {
            packages = [
              python
              pkgs.poppler-utils
              pkgs.platformio
              pkgs.gnumake
            ];
          };
        });
    };
}
