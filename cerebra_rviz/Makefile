build: cerebra-rviz/usr/share/doc/cerebra-rviz/changelog.gz
	dpkg-deb --root-owner-group --build cerebra-rviz

cerebra-rviz/usr/share/doc/cerebra-rviz/changelog.gz: changelog
	gzip -9nkc changelog > cerebra-rviz/usr/share/doc/cerebra-rviz/changelog.gz
