def allRecipientEmails = ""
def rebaseNeeded = "-1"

pipeline {
    agent any

    environment {
        REPO_NAME='sdk-alif'
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
                echo "⏳ Checking out '${REPO_NAME}' repo..."

                dir("${REPO_NAME}") {
                    checkout scm
                }

                script {
                    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                        sh '''#!/bin/bash
                        set -xe
                        export PATH=$WEST_BIN_PATH:$PATH
                        source "${ZEPHYR_4X_ENV}/environment-setup-x86_64-pokysdk-linux"
                        echo -e "🛠️ $PATH"
                        echo -e "✅ Checkout is completed "${REPO_NAME}"."

                        if [ ! -d "${REPO_NAME}" ] ; then
                            echo -e "🚫 ${REPO_NAME} directory didn't found."
                            exit -1
                        fi
                        '''
                    }
                }

                script {
                    rebaseNeeded = sh(
                        script: '''#!/bin/bash
                            set -xe
                            cd ${REPO_NAME}
                            git fetch origin refs/heads/main:refs/remotes/origin/main
                            commitCnt=$(git rev-list --count origin/main..HEAD)
                            commits=$(git log -n ${commitCnt} --pretty=format:"%H")

                            if [[ "$commits" -lt "1" ]] ; then
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

        stage('Get PR Authors') {
            steps {
                script {
                    allRecipientEmails = sh(
                        script: '''#!/bin/bash
                            set -xe
                            cd ${REPO_NAME}
                            commitCnt=$(git rev-list --count origin/main..HEAD)

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
            }
        }

        stage('west init-update and build') {
            steps {
                script {
                    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE') {
                        sh '''#!/bin/bash
                        set -xe
                        export PATH=$WEST_BIN_PATH:$PATH
                        source "${ZEPHYR_3X_ENV}/environment-setup-x86_64-pokysdk-linux"
                        echo -e "🛠️ $PATH"
                        cd "${REPO_NAME}"

                        echo -e "✅ west initialization started"
                        west init -l .
                        echo -e "☑️ west initialization is completed."
                        west update
                        echo -e "☑️ west update is completed."
                        west zephyr-export

                        commitCnt=$(git rev-list --count origin/main..HEAD)
                        totalCommitCnt=$(git rev-list --count origin/main)
                        if [[ "$totalCommitCnt" < "$commitCnt" ]] ; then
                            checkGitLog=${commitCnt}
                        else
                            checkGitLog=6
                        fi
                        git log -${checkGitLog}

                        echo -e "🎯 we have $commitCnt commits in $CHANGE_TARGET review"

                        if [[ "$commitCnt" =~ ^[0-9]+$ && "$commitCnt" -gt "0" ]] ; then
                            git format-patch -${commitCnt}
                            check_patch_output=$(./scripts/checkpatch.pl --patch *.patch)
                            echo -e "$check_patch_output"
                            check_patch_output_summary=$(echo "$check_patch_output" | grep ".patch total:")
                            echo -e "$check_patch_output_summary"

                            check_patch_errors=0
                            check_patch_warnings=0
                            check_patch_lines=0

                            while read -r line; do
                              # Extract numbers using regex or field parsing
                              err=$(echo "$line"  | awk '{for(i=1;i<=NF;i++) if($i=="total:") print $(i+1)}')
                              warn=$(echo "$line" | awk '{for(i=1;i<=NF;i++) if($i=="errors,") print $(i+1)}')
                              ln=$(echo "$line"   | awk '{for(i=1;i<=NF;i++) if($i=="warnings,") print $(i+1)}')

                              if [[ "$err" =~ ^[0-9]+$ ]]; then
                                check_patch_errors=$((check_patch_errors + err))
                              else
                                echo "Skipping non-numeric error: $err"
                              fi

                              if [[ "$warn" =~ ^[0-9]+$ ]]; then
                                check_patch_warnings=$((check_patch_warnings + warn))
                              else
                                echo "Skipping non-numeric warning: $warn"
                              fi

                              if [[ "$ln" =~ ^[0-9]+$ ]]; then
                                check_patch_lines=$((check_patch_lines + ln))
                              else
                                echo "Skipping non-numeric lines: $ln"
                              fi

                            done <<< "$check_patch_output_summary"

                            echo -e "🎯Check-Patch has=> Errors: ${check_patch_errors} Warnings: ${check_patch_warnings}, Lines: ${check_patch_lines}\n\n"

                            pr_checker_log=$(${VALIDATION_SCRIPT_DIR}/pr_commit_checker.sh  ${commitCnt})
                            pr_checker_err=$?
                            echo -e "🎯PR Checker=>\n$pr_checker_log"

                            gitlint_log=$(git log -${commitCnt} --pretty=%B | gitlint 2>&1)
                            if [ -n "$gitlint_log" ] ; then
                                gitlint_err=1
                                echo -e "🎯GITLINT log=>\n$gitlint_log"
                            else
                               gitlint_err=0
                            fi
                        else
                            gitlint_err=0
                            pr_checker_err=0
                            check_patch_errors=0
                        fi

                        #mapfile -t all_alif_boards_cfgs < <(west boards | grep "alif" | grep -v "fpga")
                        all_alif_boards_cfgs=("alif_e7_dk/ae722f80f55d5xx/rtss_he" "alif_e7_dk/ae722f80f55d5xx/rtss_hp"
                                "alif_e4_dk/ae402fa0e5597xx0/rtss_he"  "alif_e4_dk/ae402fa0e5597xx0/rtss_hp"
                                "alif_e8_dk/ae822fa0e5597xx0/rtss_he"  "alif_e8_dk/ae822fa0e5597xx0/rtss_hp")
                        supported_compilers=("gcc")
                        test_apps=("samples/hello_world")
                        total_buildError=0
                        total_cfg_run_cnt=0
                        skipped_cfg_run_cnt=0
                        actual_cfg_run_cnt=0
                        fail_cfg_run_cnt=0
                        success_cfg_run_cnt=0

                        for currCompiler in ${!supported_compilers[@]}
                        do
                            for testApp in ${!test_apps[@]}
                            do
                                for boardName in ${!all_alif_boards_cfgs[@]}
                                do
                                   total_cfg_run_cnt=$((total_cfg_run_cnt + 1))
                                   echo -e "🚩 Building for Board: ${all_alif_boards_cfgs[boardName]} AND test-app: ${test_apps[testApp]}"
                                   if [ -d build ] ; then
                                      rm -rf build
                                      echo -e "❌ Deleting Build folder..."
                                   fi

                                   actual_cfg_run_cnt=$((actual_cfg_run_cnt + 1))
                                   west build -p always -b ${all_alif_boards_cfgs[boardName]} ${test_apps[testApp]}
                                   tmp=$?
                                   echo -e "\n\n"
                                   total_buildError=$((total_buildError + tmp))
                                   echo -e "🚫 Error Status total_buildError: $total_buildError and tmp: $tmp"
                                   if [ "$tmp" -ne 0 ] ; then
                                       fail_cfg_run_cnt=$((fail_cfg_run_cnt + 1))
                                   fi
                                done
                            done
                        done
                        echo -e "✅ All Build is completed."
                        echo -e "\n\n"

                        if [[ "$total_buildError" =~ ^[0-9]+$ && "$check_patch_errors" =~ ^[0-9]+$ && "$pr_checker_err" =~ ^[0-9]+$ && "$gitlint_err" =~ ^[0-9]+$ ]]; then
                            total_jenkins_Error=$((total_buildError + check_patch_errors + pr_checker_err + gitlint_err))
                        else
                            echo "Skipping non-numeric total_buildError: $total_buildError, check_patch_errors: $check_patch_errors, pr_checker_err: $pr_checker_err, gitlint_err: $gitlint_err"
                        fi

                        echo -e " 🏆GitLint Result     =>  ${gitlint_err}"
                        echo -e " 🏆PR Checker  Result =>  ${pr_checker_err}"
                        echo -e " 🏆Check-Patch Result =>  Error: $check_patch_errors, Warnings: $check_patch_warnings"
                        echo -e " 🏆Success: $((actual_cfg_run_cnt - fail_cfg_run_cnt)), Failed: $fail_cfg_run_cnt, Skipped: $skipped_cfg_run_cnt"
                        echo -e " 🔥Total Run => ($actual_cfg_run_cnt/$total_cfg_run_cnt)"

                        exit $total_jenkins_Error
                        '''
                    }
                    echo  " 🏆Git Rebase Needed  =>  ${rebaseNeeded}"
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
                    to: "${allRecipientEmails}",
                    mimeType: 'text/html'
                )
            }
        }

        success {
            echo 'Build succeeded.'
        }

        cleanup {
            echo "Cleaning up workspace..."
            deleteDir()
        }
    }
}
