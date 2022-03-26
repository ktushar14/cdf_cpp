cd src/
clang-format -i -style=WebKit *.cpp *.hpp *.h >/dev/null 2>&1
cd - >/dev/null
cd include/covsearch/
clang-format -i -style=WebKit *.cpp *.hpp *.h >/dev/null 2>&1
cd - >/dev/null
cd include/qt/
clang-format -i -style=WebKit *.cpp *.hpp *.h >/dev/null 2>&1
cd - >/dev/null
cd tests/
clang-format -i -style=WebKit *.cpp *.hpp *.h >/dev/null 2>&1
