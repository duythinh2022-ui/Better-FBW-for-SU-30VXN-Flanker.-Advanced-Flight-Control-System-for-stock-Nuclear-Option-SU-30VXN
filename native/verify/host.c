// Runs a managed exe on the game's own Mono runtime (mono-2.0-bdwgc.dll), set up the way UnityPlayer does:
// host <mono dll> <assemblies path> <etc dir> <jit options or -> <exe> [args...]
#include <windows.h>
#include <stdio.h>
#include <string.h>
typedef struct _MonoDomain MonoDomain; typedef struct _MonoAssembly MonoAssembly;
int main(int argc, char** argv) {
  if (argc < 6) { printf("usage\n"); return 2; }
  HMODULE m = LoadLibraryA(argv[1]);
  if (!m) { printf("cannot load mono: %lu\n", GetLastError()); return 3; }
  void (*mono_set_assemblies_path)(const char*) = (void*)GetProcAddress(m, "mono_set_assemblies_path");
  void (*mono_set_dirs)(const char*, const char*) = (void*)GetProcAddress(m, "mono_set_dirs");
  void (*mono_config_parse)(const char*) = (void*)GetProcAddress(m, "mono_config_parse");
  void (*mono_jit_parse_options)(int, char**) = (void*)GetProcAddress(m, "mono_jit_parse_options");
  MonoDomain* (*mono_jit_init_version)(const char*, const char*) = (void*)GetProcAddress(m, "mono_jit_init_version");
  MonoAssembly* (*mono_domain_assembly_open)(MonoDomain*, const char*) = (void*)GetProcAddress(m, "mono_domain_assembly_open");
  int (*mono_jit_exec)(MonoDomain*, MonoAssembly*, int, char**) = (void*)GetProcAddress(m, "mono_jit_exec");
  if (!mono_set_assemblies_path || !mono_jit_init_version || !mono_domain_assembly_open || !mono_jit_exec) { printf("missing mono exports\n"); return 4; }
  if (mono_set_dirs) mono_set_dirs(argv[2], argv[3]);
  mono_set_assemblies_path(argv[2]);
  if (strcmp(argv[4], "-") != 0 && mono_jit_parse_options) { char* o[1] = { argv[4] }; mono_jit_parse_options(1, o); printf("[host] jit options: %s\n", argv[4]); }
  if (mono_config_parse) mono_config_parse(NULL);
  MonoDomain* d = mono_jit_init_version("fcs-harness", "v4.0.30319");
  if (!d) { printf("jit init failed\n"); return 5; }
  MonoAssembly* a = mono_domain_assembly_open(d, argv[5]);
  if (!a) { printf("cannot open %s\n", argv[5]); return 6; }
  int rc = mono_jit_exec(d, a, argc - 5, argv + 5);
  fflush(stdout);
  return rc;
}
