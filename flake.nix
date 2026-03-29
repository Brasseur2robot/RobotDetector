{
  description = "ESP32 development environment with PlatformIO";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      devShells.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          platformio-core
          python3
        ];

        # PlatformIO needs a writable home for its packages cache
        shellHook = ''
          exported PLATFORMIO_CORE_DIR="$HOME/.platformio"
          echo "ESP32 dev environment ready. Run 'pio run' to build."
          echo "To flash: 'pio run --target upload'"
          echo "Serial monitor: 'pio device monitor'"
        '';
      };
    });
}
