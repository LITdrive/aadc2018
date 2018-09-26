rm -r _install
rm -r _build_user
rm -r _build_user_debug
./build_base.sh && ./build_demo.sh && ./build_user.sh
