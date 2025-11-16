{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = [
    pkgs.cargo
    pkgs.pkg-config
    pkgs.systemd
    pkgs.openssl
  ];

  shellHook = 
    ''
      echo "Now run: cargo build --release"
    '';
}
