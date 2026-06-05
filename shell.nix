let
  nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-26.05";
  pkgs = import nixpkgs { config = {}; overlays = []; };
in

(pkgs.mkShell {
  buildInputs = with pkgs; [
    git
    blas
    eigen
    lapack
    python3
    python313Packages.pybind11
  ];

  nativeBuildInputs = [
    pkgs.cmake
    # Wrapper that auto-injects BLAS/LAPACK paths into every cmake invocation.
    (pkgs.writeShellScriptBin "cmake" ''
      exec ${pkgs.cmake}/bin/cmake "$@" \
        -DBLAS_LIBRARIES=${pkgs.blas}/lib/libblas.so \
        -DLAPACK_LIBRARIES=${pkgs.lapack}/lib/liblapack.so
    '')
  ];
})
