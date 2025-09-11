def rebaseNeeded       = "-1"
def allRecipientEmails = ""
def alif_board_list    = ""
def num_of_board       = ""
def prTitle            = ""
def prBody             = ""
def prSHA_ID           = ""  // Zephyr SHA
def projectName        = ""
def alif_pull_req      = ""
def hal_pull_req       = ""

def ble_samples1 = [
    ['samples/bluetooth/beacon'],             ['samples/bluetooth/broadcast_audio_sink'], ['samples/bluetooth/broadcast_audio_source'],
    ['samples/bluetooth/broadcaster'],        ['samples/bluetooth/broadcaster_multiple'], ['samples/bluetooth/central'],
    ['samples/bluetooth/central_gatt_write'], ['samples/bluetooth/central_hr'],           ['samples/bluetooth/central_ht'],
    ['samples/bluetooth/central_iso'],        ['samples/bluetooth/central_multilink']
]

def ble_samples2 = [
    ['samples/bluetooth/central_past'],  ['samples/bluetooth/direct_adv'], ['samples/bluetooth/direction_finding_central'],
    ['samples/bluetooth/direction_finding_connectionless_rx'], ['samples/bluetooth/direction_finding_connectionless_tx'],
    ['samples/bluetooth/direction_finding_peripheral'],        ['samples/bluetooth/eddystone'], ['samples/bluetooth/handsfree'],
    ['samples/bluetooth/hap_ha'],                              ['samples/bluetooth/hci_pwr_ctrl']
]

def ble_samples3 = [
    ['samples/bluetooth/ibeacon'],                 ['samples/bluetooth/ipsp'], ['samples/bluetooth/iso_broadcast'],
    ['samples/bluetooth/iso_broadcast_benchmark'], ['samples/bluetooth/iso_connected_benchmark'],
    ['samples/bluetooth/iso_receive'],             ['samples/bluetooth/mesh'], ['samples/bluetooth/mesh_demo'],
    ['samples/bluetooth/mesh_provisioner']
]

def ble_samples4 = [
    ['samples/bluetooth/observer'],        ['samples/bluetooth/periodic_adv'],           ['samples/bluetooth/periodic_sync'],
    ['samples/bluetooth/peripheral'],      ['samples/bluetooth/peripheral_accept_list'], ['samples/bluetooth/peripheral_csc'],
    ['samples/bluetooth/peripheral_dis'],  ['samples/bluetooth/peripheral_esp'],         ['samples/bluetooth/peripheral_gatt_write'],
    ['samples/bluetooth/peripheral_hids'], ['samples/bluetooth/peripheral_hr']
]

def ble_samples5 = [
    ['samples/bluetooth/peripheral_ht'],  ['samples/bluetooth/peripheral_identity'],  ['samples/bluetooth/peripheral_iso'],
    ['samples/bluetooth/peripheral_ots'], ['samples/bluetooth/peripheral_past'],      ['samples/bluetooth/peripheral_sc_only'],
    ['samples/bluetooth/scan_adv'],       ['samples/bluetooth/unicast_audio_client'], ['samples/bluetooth/unicast_audio_server']
]

def all_ble_samples = [ble_samples1, ble_samples2, ble_samples3, ble_samples4, ble_samples5]

def testApps = [
    ['samples/hello_world', 'data' ], ['samples/basic/blinky'], ['samples/drivers/watchdog'],
    ['tests/drivers/entropy/api'],    ['samples/subsys/input/input_dump'],
    //['samples/basic/blinky_pwm'],   ['samples/basic/fade_led'], ['samples/drivers/counter/alarm'],
    ['samples/drivers/uart/echo_bot'],['samples/drivers/can/counter'], ['samples/sensor/icm42670'],
    ['samples/drivers/i2s/echo'],
    //['../alif/samples/drivers/i2c_dw'], ['../alif/samples/drivers/crc'],
    //['../alif/samples/drivers/can/loopback'], ['../alif/samples/drivers/adc'],
    //['../alif/samples/drivers/dac'], ['../alif/samples/drivers/cmp'],
    //['../alif/samples/modules/tflite-micro/tflm_ethosu']
]

def buildTestApps(appList, allBoardList) {
    def stages = [:]

    for (appName in appList) {
        //This creates a new local variable inside the loop avoid Groovy variable binding issue
        def app = appName[0]
        def extraArg = appName[1]

        stages[app] = {
            echo "✅ Building  ${app}"
            catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                sh """#!/bin/bash
                set -x
                echo "🛠 PATH=> \$PATH"
                cd "\${ZEPHYR_SDK_FOLDER_NAME}"
                source "\${ZEPHYR_3X_ENV}/environment-setup-x86_64-pokysdk-linux"
                all_boards=(${allBoardList})
                overall_status=0
                totalCnt=0
                runCnt=0
                skipCnt=0
                failCnt=0
                totalCnt=\${#all_boards[@]}

                for boardName in "\${all_boards[@]}"; do
                    build_dir="build_\${boardName}_\$(basename \"${app}\")"
                    echo "🚩 Compiling for board: \$boardName, sample: ${app} (dir: \$build_dir)"

                    west build -b "\$boardName" "${app}" --build-dir "\$build_dir"
                    build_result=\$?

                    if [[ \$build_result -eq 0 ]]; then
                        echo "📌 ✅ Compilation succeeded for board: \$boardName, sample: ${app}"
                        runCnt=\$((runCnt + 1))
                    else
                        echo "❌🚫 Build failed (code: \$build_result) for board: \$boardName, sample: ${app}"
                        overall_status=1
                        failCnt=\$((failCnt + 1))
                    fi
                done

                echo "ℹ️ Run => Pass: \$runCnt/\$totalCnt, Fail: \$failCnt/\$totalCnt"
                exit \$overall_status
                """
            }
        }
    }
    return stages
}

pipeline {
    agent any

    environment {
        ALIF_SDK_FOLDER_NAME='alif'
        ALIF_SDK_REPO_URL='https://github.com/alifsemi/sdk-alif.git'
        ZEPHYR_SDK_FOLDER_NAME='zephyr'
        ZEPHYR_SDK_REPO_PATH='alifsemi/zephyr_alif'
        REF_BRANCH="main"
        PATH="${env.WEST_BIN_PATH}:${env.PATH}"
        GITHUB_TOKEN = credentials('rajranjan_github_token')
    }

    options {
        skipDefaultCheckout(true)
    }

    stages {
        stage('Install Dependencies') {
            steps {
                script {
                    sh """#!/bin/bash -xe
                    echo -e "✔️ None."
                    """
                }
            }
        }

        stage('Display Environment Variables') {
            steps {
                script {
                    sh """#!/bin/bash -xe
                        printenv
                        echo -e "✔️ Displayed all environment setting."
                    """
                }
            }
        }

        stage('checkout') {
            steps {
                cleanWs()

                echo "⏳ Checking out 📂 ${ZEPHYR_SDK_FOLDER_NAME}_alif repo..."

                dir("${ZEPHYR_SDK_FOLDER_NAME}") {
                    checkout scm
                }

                script {
                    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                        sh '''#!/bin/bash
                        set -x
                        export PATH=$WEST_BIN_PATH:$PATH
                        source "${ZEPHYR_3X_ENV}/environment-setup-x86_64-pokysdk-linux"
                        echo -e "🛠️ $PATH"
                        echo -e "✅ Checkout ${ZEPHYR_SDK_FOLDER_NAME} completed..."

                        if [ ! -d "${ZEPHYR_SDK_FOLDER_NAME}" ] ; then
                            echo -e "🚫 ${ZEPHYR_SDK_FOLDER_NAME} directory didn't found."
                            exit -1
                        fi

                        '''
                    }
                }

                script {
                    rebaseNeeded = sh(
                        script: '''#!/bin/bash
                            set -x
                            cd ${ZEPHYR_SDK_FOLDER_NAME}
                            git fetch origin refs/heads/${REF_BRANCH}:refs/remotes/origin/${REF_BRANCH}

                            if ! git rebase origin/${REF_BRANCH} ; then
                                echo -e "Rebase could not happen succesfully"
                                exit 1
                            fi

                            commitCnt=$(git rev-list --count origin/${REF_BRANCH}..HEAD)
                            commits=$(git log -n ${commitCnt} --pretty=format:"%H")

                            if [[ "$commitCnt" -lt "1" ]] ; then
                                echo 0
                                exit 0
                            fi

                            firstCommit=$(git log -n 1 --pretty=format:"%H")
                            firstCommit_author_name=$(git show -s --format="%an" "$firstCommit")
                            firstCommit_author_email=$(git show -s --format="%ae" "$firstCommit")
                            firstCommit_body=$(git show -s --format="%b" "$firstCommit" | tail -n +1 | awk '!/^[A-Za-z-]+:/ {print}' | sed '/^$/d')
                            parentCnt=$(git log -n 1 --pretty=format:"%p"  | wc -w)

                            if [[ "$firstCommit_author_name" == "Jenkins" || "$firstCommit_author_email" == "nobody@nowhere" || "$firstCommit_body" == "*Merge commit \'$firstCommit\' into HEAD*" || "$parentCnt" -gt "1" ]] ; then
                                git reset --soft HEAD~1
                                echo 1
                            else
                                echo 0
                            fi
                            ''',
                        returnStdout: true
                    ).trim()
                     echo "🎯 rebase needed: [${rebaseNeeded}]..."
                }
            }
        }

        stage('Get PR Message/Authors') {
            steps {
                script {
                    allRecipientEmails = sh(
                        script: '''#!/bin/bash
                            set -xe
                            cd ${ZEPHYR_SDK_FOLDER_NAME}
                            commitCnt=$(git rev-list --count origin/${REF_BRANCH}..HEAD)

                            # Get all author emails for this PR
                            culpritEmails=$(git log -n $commitCnt --pretty=format:'%ae' | sort | uniq | paste -sd";" -)

                            # Replace all commas with semicolons
                            defaultEmails=$(echo "$DEFAULT_OWNER_EMAILS" | tr ',' ';')

                            all_emails="$culpritEmails;$defaultEmails"
                            echo "$all_emails" | tr ';' '\n' | grep -v '^$' | sort -u | paste -sd ";" -
                        ''',
                        returnStdout: true
                    ).trim()
                    echo "📤 Mail will be sent to: [${allRecipientEmails}]..."
                }

                script {
                    withCredentials([string(credentialsId: 'rajranjan-alifsemi', variable: 'GITHUB_TOKEN')]) {
                        def response = sh(
                            script: 'curl -s -H "Authorization: token $GITHUB_TOKEN" "https://api.github.com/repos/${ZEPHYR_SDK_REPO_PATH}/pulls/${CHANGE_ID}"',
                            returnStdout: true
                        ).trim()
                        def prJson = readJSON text: response

                        prTitle    = prJson.title
                        pr_SHA_ID  = prJson.head.sha
                        prBody     = prJson.body

                        alif_pull_req = sh(
                            script: """#!/bin/bash
                                set -xe
                                echo "$prBody" | sed -n 's/.*\\*\\*alif-sdk :\\*\\* \\([0-9][0-9]*\\).*/\\1/p'
                                """,
                                returnStdout: true
                            ).trim()

                        hal_pull_req = sh(
                                script: """#!/bin/bash
                                    set -xe
                                    echo "$prBody" | sed -n 's/.*\\*\\*hal_alif :\\*\\* \\([0-9][0-9]*\\).*/\\1/p'
                                """,
                                returnStdout: true
                            ).trim()

                        echo "alif_pull_req: ${alif_pull_req}"
                        echo "hal_pull_req: ${hal_pull_req}"
                        echo "The updated SHA is: ${pr_SHA_ID}"
                        echo "PR Title:\n ${prTitle}"
                        echo "PR Body:\n ${prBody}"
                    }
                }
            }
        }

        stage('west init-update-getBoardNames') {
            steps {
                script {
                    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                        sh """#!/bin/bash
                        set -x

                        echo -e "☑️ west initialization started"
                        west init -m ${ALIF_SDK_REPO_URL} --mr main
                        echo -e "☑️ west initialization is completed."
                        west config manifest.project-filter -- +tflite-micro
                        if [[ -n "$alif_pull_req" && "$alif_pull_req" -ne 0 ]] ; then
                            cd \${ALIF_SDK_FOLDER_NAME}
                            git fetch origin pull/$alif_pull_req/head:pr-$alif_pull_req
                            git checkout pr-$alif_pull_req
                            cd ..
                        fi
                        """
                        }
                    }

                script {
                    def RE = "[[:space:]]*"
                    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                        sh """#!/bin/bash
                        set -xe
                        prj="\${ZEPHYR_SDK_FOLDER_NAME}"
                        file="\${ALIF_SDK_FOLDER_NAME}/west.yml"

                        if [ -f \${file} ]; then
                            sed -i "/^${RE}-${RE}name${RE}:${RE}\${prj}/,/^${RE}revision${RE}:${RE}/s/^\\(${RE}revision${RE}:${RE}\\).*/\\1${pr_SHA_ID}/" "\${file}"
                        else
                            echo "❌ \${file} does not exist or is not a regular file"
                        fi

                        if [[ -n "$hal_pull_req" && "$hal_pull_req" -ne 0 ]] ; then
                            cd \${ZEPHYR_SDK_FOLDER_NAME}
                            prj="hal_alif"
                            file="west.yml"
                            if [ -f \${file} ]; then
                                sed -i "/^${RE}-${RE}name${RE}:${RE}\${prj}/,/^${RE}revision${RE}:${RE}/s/^\\(${RE}revision${RE}:${RE}\\).*/\\1${pr_SHA_ID}/" "\${file}"
                            else
                                echo "❌ \${file} does not exist or is not a regular file"
                            fi
                            cd ..
                        fi

                        west update
                        echo -e "☑️ west update is completed."
                        """
                    }
                }

                script {
                    alif_board_list = sh(
                        script: '''#!/bin/bash
                            set -x
                            cd "${ZEPHYR_SDK_FOLDER_NAME}"
                            mapfile -t all_alif_boards_cfgs < <(west boards | grep "alif" | grep -v "fpga")
                            echo "${all_alif_boards_cfgs[@]}"
                        ''',
                        returnStdout: true
                    ).trim()
                    echo "⚙️ Selected Boards: [${alif_board_list}]..."

                    def gAllBoards = alif_board_list.tokenize(' ')
                    num_of_board   = gAllBoards.size()
                    echo "ℹ️ Total test apps: ${testApps.size()}"
                    echo "ℹ️ Total boards: ${num_of_board}"
                }
            }
        }

        stage('CheckPatch and Commit Message Validation') {
            steps {
                script {
                    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                        sh """#!/bin/bash
                        set -x
                        cd \${ZEPHYR_SDK_FOLDER_NAME}

                        commitCnt=\$(git rev-list --count origin/${REF_BRANCH}..HEAD)
                        totalCommitCnt=\$(git rev-list --count origin/${REF_BRANCH})

                        if [[ "\$totalCommitCnt" -lt "\$commitCnt" ]] ; then
                            checkGitLog=\${commitCnt}
                        else
                            checkGitLog=6
                        fi
                        git log -\${checkGitLog}

                        if [[ "\${commitCnt}" =~ ^[0-9]+\$ && "\${commitCnt}" -gt "0" ]] ; then
                            git format-patch -\${commitCnt}
                            check_patch_output=\$(./scripts/checkpatch.pl --patch *.patch)
                            echo -e "\$check_patch_output"
                            check_patch_output_summary=\$(echo "\$check_patch_output" | grep ".patch total:")
                            echo -e "\$check_patch_output_summary"

                            check_patch_errors=0
                            check_patch_warnings=0
                            check_patch_lines=0

                            while read -r line; do
                              # Extract numbers using regex or field parsing
                              err=\$(echo "\$line"  | awk '{for(i=1;i<=NF;i++) if(\$i=="total:") print \$(i+1)}')
                              warn=\$(echo "\$line" | awk '{for(i=1;i<=NF;i++) if(\$i=="errors,") print \$(i+1)}')
                              ln=\$(echo "\$line"   | awk '{for(i=1;i<=NF;i++) if(\$i=="warnings,") print \$(i+1)}')

                              if [[ "\$err" =~ ^[0-9]+\$ ]]; then
                                check_patch_errors=\$((check_patch_errors + err))
                              else
                                echo "Skipping non-numeric error: \$err"
                              fi

                              if [[ "\$warn" =~ ^[0-9]+\$ ]]; then
                                check_patch_warnings=\$((check_patch_warnings + warn))
                              else
                                echo "Skipping non-numeric warning: \$warn"
                              fi

                              if [[ "\$ln" =~ ^[0-9]+\$ ]]; then
                                check_patch_lines=\$((check_patch_lines + ln))
                              else
                                echo "Skipping non-numeric lines: \$ln"
                              fi

                            done <<< "\$check_patch_output_summary"

                            echo -e "🎯Check-Patch has=> Errors: \${check_patch_errors} Warnings: \${check_patch_warnings}, Lines: \${check_patch_lines}\n\n"

                            pr_checker_log=\$(${VALIDATION_SCRIPT_DIR}/pr_commit_checker.sh  \${commitCnt})
                            pr_checker_err=\$?
                            echo -e "🎯PR Checker=>\n\$pr_checker_log"

                            gitlint_log=\$(git log -\${commitCnt} --pretty=%B | gitlint 2>&1)
                            if [ -n "\$gitlint_log" ] ; then
                                gitlint_err=1
                                echo -e "🎯GITLINT log=>\n\$gitlint_log"
                            else
                               gitlint_err=0
                            fi
                        else
                            gitlint_err=0
                            pr_checker_err=0
                            check_patch_errors=0
                        fi
                        echo -e " 🏆Git Rebase Needed  =>  ${rebaseNeeded}"
                        echo -e " 🏆GitLint Result     =>  \${gitlint_err}"
                        echo -e " 🏆PR Checker  Result =>  \${pr_checker_err}"
                        echo -e " 🏆Check-Patch Result =>  Error: \$check_patch_errors, Warnings: \$check_patch_warnings"
                        """
                    }
                }
            }
        }

        stage('gcc') {
            steps {
                script {
                    try {
                        def testAppStages = buildTestApps(testApps, alif_board_list)
                        parallel testAppStages
                        if (!testAppStages || testAppStages.isEmpty()) {
                            error "❌ buildTestApps returned empty or null stages!"
                        }
                        echo "✅ Generated ${testAppStages.size()} test app stages."
                    } catch (err) {
                        error "❌ Failed to generate test app stages: ${err}"
                    }
                }
            }
        }

        stage('BLE') {
            steps {
                script {
                    for (int i = 0; i < all_ble_samples.size(); i++) {
                        def idx = i + 1 // avoid Groovy closure capture issue
                        stage("BLE Sample-${idx}") {
                            echo "Running bluetooth sample-${idx} testapp collection"
                            try {
                                def ble_samples = all_ble_samples[i]
                                def tmp_alif_board_list = "alif_b1_dk_rtss_he"
                                def testAppStages = buildTestApps(ble_samples, tmp_alif_board_list)
                                parallel testAppStages
                                if (!testAppStages || testAppStages.isEmpty()) {
                                    error "❌ buildTestApps returned empty or null stages!"
                                }
                                echo "✅ Generated ${testAppStages.size()} test app stages."
                            } catch (err) {
                                error "❌ Failed to generate test app stages: ${err}"
                            }
                        }
                    }
                }
            }
        }
    }

    post {
        failure {
            echo "🚫 Build failed. Sending email..."
            script {
                emailext (
                    subject: "🚨 Jenkins Job Failed: ${env.JOB_NAME} [${env.BUILD_NUMBER}]",
                    body: """
                       <p>Build failed for job: <b>${env.CHANGE_URL}</b></p>
                       <p>Build Number: <b>${env.BUILD_NUMBER}</b></p>
                       <p>View details: <a href="${env.BUILD_URL}">${env.BUILD_URL}</a></p>
                    """,
                    to: "raj.ranjan@alifsemi.com",
                    mimeType: 'text/html'
                )
            }
        }

        success {
            echo ' 🟢 Build succeeded...🟢'
        }

        cleanup {
            echo "Cleaning up workspace..."
            deleteDir()
        }
    }
}
