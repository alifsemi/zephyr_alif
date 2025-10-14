def clean_zephyr_cache() {
    sh """#!/bin/bash -xe
        rm -f /media/share/jenkins_share/zephyrproject_zephyr_alif.tar.gz"""
}

def zephyr_init() {
    sh """#!/bin/bash -xe
        printenv NODE_NAME
        eval `ssh-agent -s`
        rm -rf venv
        mkdir venv
        python3 -m venv venv
        . venv/bin/activate
        pip install wheel
        pip install west
        deactivate"""

    if (fileExists('/media/share/jenkins_share/zephyrproject_zephyr_alif.tar.gz')) {
        sh """#!/bin/bash -xe
            eval `ssh-agent -s`
            . venv/bin/activate
            cp /media/share/jenkins_share/zephyrproject_zephyr_alif.tar.gz .
            tar xf zephyrproject_zephyr_alif.tar.gz
            cd zephyrproject/zephyr
            west forall -c "git clean -fdx"
            west update
            west zephyr-export
            pip install -r scripts/requirements.txt
            cd ../..
            deactivate"""
    } else {
        sh """#!/bin/bash -xe
            . venv/bin/activate
            eval `ssh-agent -s`
            west init zephyrproject -m https://github.com/alifsemi/zephyr_alif.git
            cd zephyrproject/zephyr
            west forall -c "git clean -fdx"
            west update
            west zephyr-export
            pip install -r scripts/requirements.txt
            cd ../..
            tar -czvf zephyrproject_zephyr_alif.tar.gz zephyrproject
            cp zephyrproject_zephyr_alif.tar.gz /media/share/jenkins_share/
            deactivate"""
    }
}

def verify_checkpatch() {
    sh """#!/bin/bash -x
        set +e
        env
        . venv/bin/activate
        cd zephyrproject/zephyr
        git fetch --all
        if [ -n ${CHANGE_BRANCH} ];
          then git checkout $CHANGE_BRANCH;
        else
          echo "building 'default' branch";
        fi

	GIT_REMOTE=`git remote`
	echo \$GIT_REMOTE

        if ./scripts/checkpatch.pl --ignore=GERRIT_CHANGE_ID,EMAIL_SUBJECT,COMMIT_MESSAGE,COMMIT_LOG_LONG_LINE -g \${GIT_REMOTE}/\${CHANGE_BRANCH}...\${GIT_REMOTE}/main ;
            then echo 'Checkpatch OK'
        else
            echo 'ERROR:scripts/checkpatch.pl detected errors/warnings.'; echo 'CMD:Run git diff --cached | ./scripts/checkpatch.pl - before committing' ; exit 1 ;
        fi
        deactivate"""
}

def build_zephyr(String sample, String build_dir, String board, String conf_file=null) {
    //if (!sample or !build_dir or !board) {
    //	error "Failed, null arguments given... No reason to continue execution"
    //}
    println(sample)
    println(build_dir)
    println(board)
    println(conf_file)
    String overlay = ""
    if (conf_file) {
    	overlay = "-- -DOVERLAY_CONFIG=$conf_file"
    }
    sh """#!/bin/bash -x
        set +e
        env
        . venv/bin/activate
        cd zephyrproject/zephyr
        git fetch --all
        if [ -n ${CHANGE_BRANCH} ];
          then git checkout $CHANGE_BRANCH;
        else
          echo "building 'default' branch";
        fi
        west build -p always -b $board --build-dir $build_dir $sample $overlay
        deactivate"""
        //manual fail
        //error "Failed, Maintenance Mode not Enabled... No reason to continue execution"
}

def run_test(String jsonfile, String zephyr_build_dir, String test) {

    sh """#!/bin/bash -xe
        printenv NODE_NAME
        sed -i "s/ttyUSB.*/ttyAlifSEDUT1/g" $ALIF_SETOOLS_ORIG/isp_config_data.cfg
        rsync -a --delete $ALIF_SETOOLS_ORIG $ALIF_SETOOLS_LOCATION
        cp $zephyr_build_dir/zephyr.bin $ALIF_SETOOLS_LOCATION/build/images/
        cp $jsonfile $ALIF_SETOOLS_LOCATION/build/config/
        pushd $ALIF_SETOOLS_LOCATION/
        ./app-gen-toc --filename build/config/$jsonfile
        ./app-write-mram -p
        popd"""

    sh """#!/bin/bash -xe
        pushd pytest
        export PATH=$PATH:$JLINK
        sed -e "s/ttyUSB2/ttyAlifSE$DUT_NUMBER/g" -e "s/ttyUSB1/ttyAlifUART2$DUT_NUMBER/g" -e "s/ttyUSB0/ttyAlifUART4$DUT_NUMBER/g" pytest.ini.template > pytest.ini
        cat pytest.ini
        mkdir junit
        mkdir pytest-logs
        mkdir venv
        python3 -m venv venv
        . venv/bin/activate
        pip install -r requirements.txt
        pytest -s -k $test --tb=native --junit-xml junit/zephyr-junit-report.xml --root-logdir=pytest-logs
        deactivate
        popd
        """
}


return [
    build_zephyr: this.&build_zephyr,
    run_test: this.&run_test,
    clean_zephyr_cache: this.&clean_zephyr_cache,
    zephyr_init: this.&zephyr_init,
    verify_checkpatch: this.&verify_checkpatch
]
