import groovy.json.JsonOutput
import groovy.json.JsonSlurper

private def request(String method, String path, Object body = null, String token) {
    String github_api = "https://api.github.com"
    String json       = body ? JsonOutput.toJson(body).replace("'", "'\\''") : ""
    String data       = body ? "-d ${json}" : ""
    String cmd        = ""

    cmd += "curl -sS "
    cmd += "-X ${method} "
    cmd += "-H 'Accept: application/vnd.github+json' "
    cmd += "-H 'X-GitHub-Api-Version: 2026-03-10' "

    if (body != null) {
        cmd += "${data} "
    }

    cmd += ' "' + github_api + path + '"'

    def response = sh(
        script: """\
            set +x
            ${cmd} -H "Authorization: Bearer \$GITHUB_TOKEN" -w "\\nHTTP_STATUS:%{http_code}"
        """,
        returnStdout: true
    ).trim()

    if (!response) {
        echo "Empty response from GitHub"
        return [:]
    }

    def lines    = response.readLines()
    def status   = lines[-1].replace("HTTP_STATUS:", "").trim()
    def respBody = lines.size() > 1 ? lines[0..-2].join("\n") : ""

    int httpStatus = status.toInteger()

    if (httpStatus < 200 || httpStatus >= 300) {
        echo "WARNING: GitHub request API failed. HTTP ${status}"
        echo "Response: ${respBody}"
    } else {
        echo "HTTP Status: ${status}"
    }

    // DELETE normally returns 204 with no response body
    if (httpStatus == 204 || !respBody) {
        return [:]
    }

    try {
        return new JsonSlurper().parseText(respBody)
    } catch (Exception e) {
        echo "Response is not JSON"
        return respBody
    }
}

def createPullRequest(Map a) {
    String body= a.body ?: ""
    String argJson = '{' +
        '"title": "' + a.title + '", ' +
        '"head" : "' + a.head  + '", ' +
        '"base" : "' + a.base  + '", ' +
        '"body" : "' + a.body  + '"'   +
        '}'

    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/pulls",
        argJson,
        a.token
    )
}

def getPullRequest(Map a) {
    request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.prNumber}",
        null,
        a.token
    )
}

def listPullRequests(Map a) {
    def prs     = []
    int page    = 1
    int perPage = 100

    while (true) {
        def git_api_path =
            "/repos/${a.owner}/${a.repo}/pulls?state=${a.state ?: 'open'}&per_page=${perPage}&page=${page}"

        echo "Fetching PR page: ${page}"

        def result = request(
            "GET",
            git_api_path,
            null,
            a.token
        )

        if (!(result instanceof List)) {
            echo "Unexpected response while fetching PRs"
            return prs
        }

        echo "PRs returned in page ${page}: ${result.size()}"

        for (def pr : result) {
            prs << [
                prNumber: pr.number,
                body    : pr.body?.toString(),
                title   : pr.title?.toString(),
                state   : pr.state?.toString()
            ]
        }

        // Less than 100 means this was the last page
        if (result.size() < perPage) {
            break
        }
        page++
    }

    echo "Total PRs collected: ${prs.size()}"
    return prs
}

def updatePullRequest(Map a) {
    request(
        "PATCH",
        "/repos/${a.owner}/${a.repo}/pulls/${a.prNumber}",
        a.body,
        a.token
    )
}

def getPullRequestTitle(Map a) {
    def result = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.prNumber}",
        null,
        a.token
    )

    if (result instanceof Map && result.title != null) {
        return result.title.toString()
    }

    return ""
}

def getPullRequestBody(Map a) {
    def result = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.prNumber}",
        null,
        a.token
    )

    if (result instanceof Map && result.body != null) {
        return result.body.toString()
    }

    return ""
}

def addLabel(Map a) {
    String argJson = '[ "' + a.label + '" ]'
    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/issues/${a.prNumber}/labels",
        argJson,
        a.token
    )
}

def removeLabel(Map a) {
    request(
        "DELETE",
        "/repos/${a.owner}/${a.repo}/issues/${a.prNumber}/labels/${a.label}",
        null,
        a.token
    )
}

def getPRlabelsList(Map a) {
    request(
        "GET",
        "/repos/${a.owner}/${a.repo}/issues/${a.prNumber}/labels",
        null,
        a.token
    )
}

def addComment(Map a) {
    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/issues/${a.prNumber}/comments",
        ['"body" : "' + a.comment + '"'],
        a.token
    )
}

def updateComment(Map a) {
    request(
        "PATCH",
        "/repos/${a.owner}/${a.repo}/issues/comments/${a.commentId}",
        ['"body" : "' + a.comment + '"'],
        a.token
    )
}

def getCommentID(Map a, String commentToBeSearch) {
    def commentId = null

    def getAllComments = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/issues/${a.prNumber}/comments",
        null,
        a.token
    )

    if (!(getAllComments instanceof List)) {
        echo "Failed to get comments for PR #${prNumber} from GitHub"
        return null
    }

    getAllComments.each { comment ->
        if (comment.body?.contains(commentToBeSearch)) {
            commentId = comment.id
        }
    }

    return commentId
}

def deleteComment(Map a) {
    request(
        "DELETE",
        "/repos/${a.owner}/${a.repo}/issues/comments/${a.commentId}",
        null,
        a.token
    )
}

def getPullRequestSha(Map a) {
    def result = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.prNumber}",
        null,
        a.token
    )

    return result?.head?.sha
}

def getPullRequestCommitMessage(Map a) {
    def result = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.prNumber}/commits?per_page=100",
        null,
        a.token
    )

    if (!(result instanceof List) || result.isEmpty()) {
        return null
    }

    return result[-1]?.commit?.message
}

def getAllPRFiles(Map a, String prNumber) {
    def allFiles = []
    int page = 1
    int perPage = 100

    while (true) {
        def files = request(
            "GET",
            "/repos/${a.owner}/${a.repo}/pulls/${prNumber}/files?per_page=${perPage}&page=${page}",
            null,
            a.token
        )

        if (!(files instanceof List) || files.isEmpty()) {
            break
        }

        allFiles.addAll(files)

        // If less than 100 were returned, this was the last page
        if (files.size() < perPage) {
            break
        }

        page++
    }

    return allFiles
}

def findPRsWithDiff(Map a, String searchString) {
    def matchingPRs = []
    def allFiles    = []
    int page        = 1
    int perPage     = 100
    int fPage       = 1
    int fPerPage    = 100

    while (true) {
        def git_api_path =
            "/repos/${a.owner}/${a.repo}/pulls?state=${a.state ?: 'open'}&per_page=${perPage}&page=${page}"

        echo "Fetching PR page: ${page}"

        def prs = request(
            "GET",
            git_api_path,
            null,
            a.token
        )

        if (!(prs instanceof List)) {
            echo "Unexpected response while fetching PRs"
            return matchingPRs
        }

        echo "PRs returned in page ${page}: ${prs.size()}"
        prs.each { pr ->
            def prNumber = pr.number
            echo "Checking PR #${prNumber}"

            def files = getAllPRFiles(a, prNumber)

            files.each { file ->
                def patch = file.patch ?: ""
                if (patch.contains(searchString)) {
                    echo "Found '${searchString}' in PR #${prNumber}"
                    echo "File: ${file.filename}"

                    matchingPRs << [
                        prNumber: prNumber,
                        title   : pr.title,
                        filename: file.filename,
                        patch   : patch
                    ]
                }
            }
        }

        // Less than 100 means this was the last page
        if (prs.size() < perPage) {
            break
        }
        page++
    }

    echo "Total PRs collected: ${matchingPRs.size()}"
    return matchingPRs

}

return this
