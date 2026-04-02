# Helper script invoked at build time to update a ROOT dictionary stamp file
# only when the content of the input headers has actually changed.
#
# This breaks the mtime-based dependency chain for rootcling: rather than
# rebuilding dictionaries whenever any header has a newer mtime (e.g., after
# git pull or cmake reconfigure), rootcling only reruns when header content
# actually differs.
#
# Arguments (passed via -D flags):
#   HEADERS_LIST_FILE  -- path to a cmake file that sets DICT_HEADERS to the
#                         list of input header paths
#   STAMP_FILE         -- path to the stamp file to (conditionally) update

if(NOT DEFINED HEADERS_LIST_FILE OR NOT DEFINED STAMP_FILE)
  message(FATAL_ERROR
    "update_dict_stamp.cmake requires HEADERS_LIST_FILE and STAMP_FILE variables")
endif()

if(NOT EXISTS "${HEADERS_LIST_FILE}")
  message(FATAL_ERROR
    "Headers list file missing: ${HEADERS_LIST_FILE}\n"
    "Re-run cmake to regenerate it: cmake -S <src> -B <build>")
endif()

include("${HEADERS_LIST_FILE}") # sets DICT_HEADERS

# Compute combined SHA256 hash of all input headers.
set(_combined "")
foreach(_h IN LISTS DICT_HEADERS)
  if(NOT EXISTS "${_h}")
    message(FATAL_ERROR "Header not found: ${_h}")
  endif()
  file(SHA256 "${_h}" _h_hash)
  string(APPEND _combined "${_h}:${_h_hash}\n")
endforeach()
string(SHA256 _new_hash "${_combined}")
set(_new_content "${_new_hash}\n")

# Only rewrite stamp file when the hash has changed.
# Preserving the mtime when content is unchanged prevents rootcling from
# rerunning (the dictionary custom command depends on this stamp's mtime).
set(_old_content "")
if(EXISTS "${STAMP_FILE}")
  file(READ "${STAMP_FILE}" _old_content)
endif()
if(NOT _new_content STREQUAL _old_content)
  file(WRITE "${STAMP_FILE}" "${_new_content}")
endif()
