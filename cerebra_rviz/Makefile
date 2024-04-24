SHELL = /bin/bash

build/cerebra-rviz.deb: build/cerebra-rviz.stamp build/cerebra-rviz/usr/bin/cerebra-rviz build/cerebra-rviz/DEBIAN/control build/cerebra-rviz/usr/share/doc/cerebra-rviz/changelog.gz build/cerebra-rviz/usr/share/man/man1/cerebra-rviz.1.gz
	mkdir -p build
	dpkg-deb --root-owner-group --build build/cerebra-rviz

build/cerebra-rviz.stamp: $(shell find cerebra-rviz/*)
	mkdir -p build/cerebra-rviz
	cp -r cerebra-rviz build/
	touch build/cerebra-rviz.stamp

build/cerebra-rviz/usr/bin/cerebra-rviz: cerebra-rviz.in VERSION
	mkdir -p build/cerebra-rviz/usr/bin
	sed -e "s/@VERSION@/$(shell cat VERSION)/" cerebra-rviz.in > build/cerebra-rviz/usr/bin/cerebra-rviz
	chmod +x build/cerebra-rviz/usr/bin/cerebra-rviz

build/cerebra-rviz/DEBIAN/control: control.in VERSION
	mkdir -p build/cerebra-rviz/DEBIAN
	sed -e "s/@VERSION@/$(shell cat VERSION)/" control.in > build/cerebra-rviz/DEBIAN/control

build/cerebra-rviz/usr/share/doc/cerebra-rviz/changelog.gz: changelog
	mkdir -p build/cerebra-rviz/usr/share/doc/cerebra-rviz
	gzip -9nkc changelog > build/cerebra-rviz/usr/share/doc/cerebra-rviz/changelog.gz

build/cerebra-rviz.1: build/cerebra-rviz/usr/bin/cerebra-rviz VERSION
	mkdir -p build
	help2man --no-info --name="Cerebra RViz" --version-string="$(shell cat VERSION)" --output=build/cerebra-rviz.1 build/cerebra-rviz/usr/bin/cerebra-rviz

build/cerebra-rviz/usr/share/man/man1/cerebra-rviz.1.gz: build/cerebra-rviz.1
	mkdir -p build/cerebra-rviz/usr/share/man/man1
	gzip -9nkc build/cerebra-rviz.1 > build/cerebra-rviz/usr/share/man/man1/cerebra-rviz.1.gz

clean:
	rm -rf build
